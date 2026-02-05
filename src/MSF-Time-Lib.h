#include <Arduino.h>
#include <TimeLib.h> // https://github.com/PaulStoffregen/Time

#ifndef MSF_TIME_LIB_DEBUG
  #define MSF_TIME_LIB_DEBUG 1
#endif

#if MSF_TIME_LIB_DEBUG
  #define MSF_LOG(...)   Serial.print(__VA_ARGS__)
  #define MSF_LOGLN(...) Serial.println(__VA_ARGS__)
#else
  #define MSF_LOG(...)
  #define MSF_LOGLN(...)
#endif


#define WARMUP_TIME_MS 0 

#define WARMUP_TIME_MS 0 


struct MSFData {
  tmElements_t time;    
  bool checksum_passed;
  int sync_quality;    
  int bit_certainty;   
};

template <int SAMPLE_RATE_MS>
class MSFReceiver {
  
  using ReaderFunction = bool (*)();

  private:
    static const int BUFFER_SIZE     = 1500 / SAMPLE_RATE_MS; 
    // Calculate bytes needed: (1500 + 7) / 8 = 188 bytes
    static const int BUFFER_BYTES    = (BUFFER_SIZE + 7) / 8;
    
    static const int SAMPLES_SILENCE = 500  / SAMPLE_RATE_MS; 
    static const int SAMPLES_CARRIER = 700  / SAMPLE_RATE_MS; 
    static const int LOOKBACK_TOTAL  = SAMPLES_CARRIER + SAMPLES_SILENCE;

    ReaderFunction _reader;
    
    // --- RAM OPTIMIZATION: BIT PACKING ---
    uint8_t packedA[8]; 
    uint8_t packedB[8]; 
    
    // Now takes 188 bytes instead of 1500 bytes
    uint8_t buffer[BUFFER_BYTES]; 
    
    int head;                 
    int carrierWindowScore;   
    int silenceWindowScore; 
    int _lastSyncScore;  

    // --- BITWISE HELPERS ---
    
    // Set a bit in the circular buffer
    void setBufferBit(int index, bool value) {
      if (value) buffer[index / 8] |= (1 << (index % 8));
      else       buffer[index / 8] &= ~(1 << (index % 8));
    }

    // Read a bit from the circular buffer
    bool getBufferBit(int index) {
      return (buffer[index / 8] >> (index % 8)) & 1;
    }

    void writeBit(uint8_t* array, int index, bool value) {
      if (index < 0 || index >= 60) return;
      int byteIndex = index / 8;
      int bitIndex  = index % 8;
      if (value) array[byteIndex] |= (1 << bitIndex);
      else       array[byteIndex] &= ~(1 << bitIndex);
    }

    bool readBit(uint8_t* array, int index) {
      if (index < 0 || index >= 60) return 0;
      int byteIndex = index / 8;
      int bitIndex  = index % 8;
      return (array[byteIndex] >> bitIndex) & 1;
    }

    // --- SCORING (O(1)) ---
    int updateRollingBuffer(bool carrierState) {
      int idx_500ms_ago = head - SAMPLES_SILENCE;
      if (idx_500ms_ago < 0) idx_500ms_ago += BUFFER_SIZE;
      int idx_1200ms_ago = head - LOOKBACK_TOTAL;
      if (idx_1200ms_ago < 0) idx_1200ms_ago += BUFFER_SIZE;

      // Use helper to read packed bits
      bool bit_moving_to_carrier = getBufferBit(idx_500ms_ago); 
      bool bit_falling_off_edge = getBufferBit(idx_1200ms_ago);

      if (carrierState == 0) silenceWindowScore++;
      if (bit_moving_to_carrier == 0) silenceWindowScore--;
      if (bit_moving_to_carrier == 1) carrierWindowScore++;
      if (bit_falling_off_edge == 1) carrierWindowScore--;

      // Use helper to write packed bit
      setBufferBit(head, carrierState);
      
      head++;
      if (head >= BUFFER_SIZE) head = 0;

      return carrierWindowScore + silenceWindowScore;
    }

    int decodeBCD(int startIdx, int count, const int* weights) {
      int val = 0;
      for (int i = 0; i < count; i++) {
        if (startIdx + i >= 60) break; 
        if (readBit(packedA, startIdx + i)) val += weights[i];
      }
      return val;
    }

    bool checkParity(int startIdx, int count, int parityBitIdx) {
      int ones = 0;
      for (int i = 0; i < count; i++) {
        if (readBit(packedA, startIdx + i)) ones++;
      }
      if (readBit(packedB, parityBitIdx)) ones++;
      return (ones % 2 != 0); 
    }

    // --- SYNC ENGINE ---
    uint32_t syncToPulse() {
      delay(random(2000, 5000)); 
      
      MSF_LOGLN(F("[MSF] Syncing... (Scanning 65s for Minute Marker)"));
      
      head = 0;
      // Initialize all bits to 1 (Steady Carrier). 0xFF is 11111111 in binary.
      memset(buffer, 0xFF, sizeof(buffer)); 
      
      silenceWindowScore = 0;            
      carrierWindowScore = SAMPLES_CARRIER;

      uint32_t startScan = millis();
      uint32_t lastSample = 0;
      uint32_t lastPrint = 0;
      int maxScoreSeen = 0; 
      int lastCalculatedScore = 0;
      uint32_t timeOfMaxScore = 0;

      while (millis() - startScan < 65000) {
        uint32_t now = millis();
        if (now - lastSample >= SAMPLE_RATE_MS) {
          lastSample = now;
          int currentScore = updateRollingBuffer(_reader()); 
          lastCalculatedScore = currentScore;
          
          if (now - startScan > WARMUP_TIME_MS) {
            if (currentScore > maxScoreSeen) {
              maxScoreSeen = currentScore;
              timeOfMaxScore = now;
            }
          }
        }
        
        if (now - lastPrint >= 100) {
            lastPrint = now;
            MSF_LOG(F("\r[MSF] T+"));
            MSF_LOG((now - startScan) / 1000);
            MSF_LOG(F("."));
            MSF_LOG(((now - startScan) % 1000) / 100);
            MSF_LOG(F("s | Curr: "));
            MSF_LOG(lastCalculatedScore);
            MSF_LOG(F(" | Best: "));
            MSF_LOG(maxScoreSeen);
            MSF_LOG(F("         ")); 
        }
      }
      
      MSF_LOGLN(); 
      _lastSyncScore = maxScoreSeen;
      MSF_LOG(F("[MSF] Final Peak Score: ")); MSF_LOG(maxScoreSeen); MSF_LOGLN();

      return timeOfMaxScore - 500; 
    }

  public:
    MSFReceiver(ReaderFunction readerFunc) 
      : _reader(readerFunc), head(0), carrierWindowScore(0), silenceWindowScore(0), _lastSyncScore(0) {}

    // --- GET TIME ---
    MSFData get_time() {
      // 1. SYNC
      uint32_t prevMinuteMillis = syncToPulse();
      uint32_t nextMinuteMillis = prevMinuteMillis + 60000;

      while (millis() >= nextMinuteMillis) {
         nextMinuteMillis += 60000;
         long wait_ms = (long)(nextMinuteMillis - millis());
         MSF_LOG(F("[MSF] Missed the train! Skipping to next minute, will wait "));
         MSF_LOG(wait_ms);
         MSF_LOGLN(F("ms..."));
      }

      MSF_LOG(F("[MSF] Waiting for next minute boundary..."));
      while (millis() < nextMinuteMillis) { delay(1); }
      MSF_LOGLN(F(" NOW."));
      
      // 2. READ DATA
      uint32_t minuteStart = nextMinuteMillis;
      uint32_t nextSecondBoundary = 1000;
      uint32_t lastSample = 0;
      
      int countHighA = 0, totalA = 0;
      int countHighB = 0, totalB = 0;
      int currentSec = 0;
      long totalCertaintyAccumulator = 0; 

      memset(packedA, 0, sizeof(packedA));
      memset(packedB, 0, sizeof(packedB));

      MSF_LOGLN(F("[MSF] ------------------------------------------------"));
      MSF_LOGLN(F("[MSF] SEC |   BIT A (135-165ms)   |   BIT B (235-265ms)"));
      MSF_LOGLN(F("[MSF] ------------------------------------------------"));

      while (currentSec < 60) {
        uint32_t now = millis();
        uint32_t elapsed = now - minuteStart;

        if (elapsed >= nextSecondBoundary) {
            int pctA = (totalA > 0) ? (countHighA * 100) / totalA : 0;
            int pctB = (totalB > 0) ? (countHighB * 100) / totalB : 0;
            
            bool valA = (pctA > 60);
            bool valB = (pctB > 60);
            writeBit(packedA, currentSec, valA);
            writeBit(packedB, currentSec, valB);
            
            int certA = abs(50 - pctA) * 2;
            int certB = abs(50 - pctB) * 2;
            totalCertaintyAccumulator += (certA + certB);

            MSF_LOG(F("[MSF] Sec ")); 
            if(currentSec < 10) MSF_LOG(F("0")); MSF_LOG(currentSec);
            MSF_LOG(F(" | A:")); MSF_LOG(valA); MSF_LOG(F(" [")); MSF_LOG(pctA); MSF_LOG(F("%]"));
            MSF_LOG(F(" | B:")); MSF_LOG(valB); MSF_LOG(F(" [")); MSF_LOG(pctB); MSF_LOG(F("%]"));
            
            if (pctA < 90 && pctA > 10) MSF_LOG(F(" <--- NOISY"));
            MSF_LOGLN();

            currentSec++;
            nextSecondBoundary += 1000;
            countHighA = 0; totalA = 0;
            countHighB = 0; totalB = 0;
        }

        if (now - lastSample >= SAMPLE_RATE_MS) {
            lastSample = now;
            int ms = elapsed % 1000;
            bool pinState = _reader(); 
            bool isSilence = !pinState; 

            if (ms >= 135 && ms <= 165) {
                totalA++;
                if (isSilence) countHighA++;
            }
            else if (ms >= 235 && ms <= 265) {
                totalB++;
                if (isSilence) countHighB++;
            }
        }
      }
      MSF_LOGLN(F("[MSF] ------------------------------------------------"));

      // 3. DECODE
      MSFData result;
      result.sync_quality = _lastSyncScore;
      result.bit_certainty = totalCertaintyAccumulator / 120; 

      result.time.Second = 0; 
      result.time.Minute = 0;
      result.time.Hour   = 0;
      result.time.Day    = 1;
      result.time.Month  = 1;
      result.time.Year   = 0; 

      static const int wYear[]  = {80, 40, 20, 10, 8, 4, 2, 1}; 
      static const int wMonth[] = {10, 8, 4, 2, 1};             
      static const int wDay[]   = {20, 10, 8, 4, 2, 1};         
      static const int wDOW[]   = {4, 2, 1};                    
      static const int wHour[]  = {20, 10, 8, 4, 2, 1};         
      static const int wMin[]   = {40, 20, 10, 8, 4, 2, 1};     

      int rawYear = decodeBCD(17, 8, wYear); 
      result.time.Year   = rawYear + 30; 
      result.time.Month  = decodeBCD(25, 5, wMonth);
      result.time.Day    = decodeBCD(30, 6, wDay);
      result.time.Hour   = decodeBCD(39, 6, wHour);
      result.time.Minute = decodeBCD(45, 7, wMin);
      result.time.Wday   = decodeBCD(36, 3, wDOW) + 1; 

      bool pYear = checkParity(17, 8, 54);
      bool pDate = checkParity(25, 11, 55);
      bool pDOW  = checkParity(36, 3, 56);
      bool pTime = checkParity(39, 13, 57);
      
      bool sane  = (result.time.Month >= 1 && result.time.Month <= 12) && 
                   (result.time.Day >= 1 && result.time.Day <= 31) &&
                   (result.time.Hour <= 23) && (result.time.Minute <= 59);

      result.checksum_passed = pYear && pDate && pDOW && pTime && sane;
      
      return result;
    }

    MSFData get_time_with_retry() {
      while (true) {
        MSF_LOGLN(F("\n[MSF] Attempting to acquire atomic time..."));
        MSFData res = get_time();
        
        if (res.checksum_passed) {
          MSF_LOGLN(F("[MSF] SUCCESS! Checksum Passed."));
          return res;
        } else {
          MSF_LOGLN(F("[MSF] Checksum Failed. Retrying..."));
        }
      }
    }
};
