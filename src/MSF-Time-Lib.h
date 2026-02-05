#include <Arduino.h>
#include <TimeLib.h> // https://github.com/PaulStoffregen/Time

#ifndef MSF_TIME_LIB_DEBUG
#define MSF_TIME_LIB_DEBUG 1
#endif

#if MSF_TIME_LIB_DEBUG
#define MSF_LOG(...) Serial.print(__VA_ARGS__)
#define MSF_LOGLN(...) Serial.println(__VA_ARGS__)
#else
#define MSF_LOG(...)
#define MSF_LOGLN(...)
#endif

struct MSFData
{
  tmElements_t time;
  bool checksum_passed;
};

template <int SAMPLE_RATE_MS>
class MSFReceiver
{
  using ReaderFunction = bool (*)();

private:
  // TODO: 1500 ms actually gives us 300ms of unmonitored zone, we can probably reduce this to 1200ms to
  // fit exactly the carrier and silence windows with no unmonitored zone.
  // This was just some lazy way to make binary safety, see docstring in updateRollingBuffer function for more details.
  // Will be eventually removed to save memory
  static const int MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_NUM_ELEMENTS = 1500 / SAMPLE_RATE_MS;
  static const int MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_BYTES = (MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_NUM_ELEMENTS + 7) / 8;

  // if you look at MSF spec document at: https://www.pvelectronics.co.uk/rftime/msf/MSF_Time_Date_Code.pdf
  // you can see that the minute is clearly identifiable by last 59th second having 700ms of carrier
  // followed by 500ms of silence in 0th second of the next minute.
  // We are going to look for this tranistion, keep its timestamp and then wait for the next minute boundary to start reading the bits.
  static const int MINUTE_MARKER_NUM_SAMPLES_CARRIER = 700 / SAMPLE_RATE_MS;
  static const int MINUTE_MARKER_NUM_SAMPLES_SILENCE = 500 / SAMPLE_RATE_MS;
  static const int LOOKBACK_TOTAL = MINUTE_MARKER_NUM_SAMPLES_CARRIER + MINUTE_MARKER_NUM_SAMPLES_SILENCE;

  // we need to store 60 boolean bit for A and B msf payloads, but bool datatype takes a whole byte
  // to save memory on smaller platforms, we can pack 8 bits into a single uint8_t
  // there is helper function below to read/write individual bits from these packed arrays
  // NOTE: do not read/write directly to these arrays, use the helper functions to set/get bits.
  uint8_t packedA[8];
  uint8_t packedB[8];

  // same as above, when we are listening for a minute marker we want to store 1.5 seconds of data representing
  // transition between 59th second and 0th second of new minute.
  // this is again 1500ms / SAMPLE_RATE_MS boolean samples, which we pack into uint8_t to save memory.
  // This is a rolling buffer as we never need to store the entire 60s of data
  // NOTE: do not read/write directly to this buffer, use the helper functions to set/get bits and update the rolling buffer score.
  uint8_t buffer[MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_BYTES];

  // State variables for syncing to the minute marker.
  int rollingBufferHead;
  int rollingBufferCarrierWindowScore;
  int rollingBufferSilenceWindowScore;

  // reader function provided by the user code to read the current state of the carrier (true for carrier, false for silence)
  ReaderFunction _reader;

  /// @brief Helper function used to write a single bit into a packed uint8_t array
  /// @param array the array to write into
  /// @param index the index of the bit to write (0-based)
  /// @param value the boolean value to write
  void writeBit(uint8_t *array, int index, bool value)
  {
    if (value)
      array[index / 8] |= (1 << (index % 8));
    else
      array[index / 8] &= ~(1 << (index % 8));
  }
  /// @brief Helper function used to read a single bit from a packed uint8_t array
  /// @param array the array to read from
  /// @param index the index of the bit to read (0-based)
  /// @return the boolean value of the bit
  bool readBit(const uint8_t *array, int index)
  {
    return (array[index / 8] >> (index % 8)) & 1;
  }
  /// @brief Helper function used to update our rolling buffer which pushes a new sample in and returns the confidence score of us being currently at the minute marker transition based on how many samples in the carrier and silence windows are correct according to the MSF spec.
  /// @param carrierValue Boolean value representing the new sample we are pushing into our rolling buffer, true for carrier and false for silence. This should be the latest reading from our input pin at the current time, which is typically read at a regular interval defined by SAMPLE_RATE_MS.
  /// @return Integer score representing how confident we are that we are currently looking at the minute marker transition, where higher score means more confidence. The score is calculated based on how many samples in our carrier and silence windows match the expected pattern for a minute marker transition according to the MSF spec.
  int updateRollingBuffer(bool carrierValue)
  {
    // this is how the perfect minute marker looks like
    // |<- 700ms carrier      ->|<- 500ms silence ->|
    // our roling data structure is 1500ms long and we are pushing in the data from the left to right
    // |<- our carrier region ->|<- our silence region ->|<- unmonitored zone ->|
    // we track how many samples in our carrier region are actually carrier and how many samples in our silence region are actually silencee
    // and we report that score on the return, where higher score means more confidence that we are currently looking at the minute marker transition right now.
    // NOTE: unmonitored zone is just for binary safety, will be removed later

    // identify the two samples that are about to leave their windoes (silence, carrier)

    // find the index of the sample thats about to go from silence window to carrier window
    int indexAtSilenceEdge = this->rollingBufferHead - MINUTE_MARKER_NUM_SAMPLES_SILENCE;
    if (indexAtSilenceEdge < 0)
      indexAtSilenceEdge += MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_NUM_ELEMENTS;

    // find the index of the sample thats about to fall off the carrier window into our unmonitored zone
    int indexAtCarrierEdge = this->rollingBufferHead - LOOKBACK_TOTAL;
    if (indexAtCarrierEdge < 0)
      indexAtCarrierEdge += MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_NUM_ELEMENTS;

    // read the samples at these two edges before we overwrite the rollingBufferHead with the new sample
    bool sampleLeavingSilence = this->readBit(this->buffer, indexAtSilenceEdge);
    bool sampleLeavingCarrier = this->readBit(this->buffer, indexAtCarrierEdge);

    // --- CARRIER REGION UPDATES ---

    // if the sample that is leaving the silence window into carrier window is carrier its good for our minute marker, so we increase the score
    if (sampleLeavingSilence == 1)
      this->rollingBufferCarrierWindowScore++;

    // if the sample that is leaving our carrier window is carrier, we are losing a good sample for our minute marker, so we decrease the score
    if (sampleLeavingCarrier == 1)
      this->rollingBufferCarrierWindowScore--;

    // --- SILENCE REGION UPDATES ---

    // the new sample that is entering the silence window, if its is silence (0) it is good for our minute marker, so we increase the score
    if (carrierValue == 0)
      this->rollingBufferSilenceWindowScore++;

    // if sample that is leaving silence is silence (0) we are losing a good sample for our minute marker, so we decrease the score
    if (sampleLeavingSilence == 0)
      this->rollingBufferSilenceWindowScore--;

    // push the new sample into the buffer, this will overwrite the sample at rollingBufferHead, but we have already accounted for that sample leaving the windows and updated our scores accordingly
    this->writeBit(this->buffer, this->rollingBufferHead, carrierValue);

    // if we reached the end of our buffer go back in circle
    this->rollingBufferHead++;
    if (this->rollingBufferHead >= MINUTE_MARKER_LOOKUP_BUFFER_SIZE_IN_NUM_ELEMENTS)
      this->rollingBufferHead = 0;

    // return both scores togather, where a top score is good carrier window and good silence window
    return this->rollingBufferCarrierWindowScore + this->rollingBufferSilenceWindowScore;
  }

  /// @brief Helper function that cleans up our rolling buffer global variabes and initializes them to be ready for next syn attempt
  void rollingBufferSetupAndCleanup()
  {
    this->rollingBufferHead = 0;
    memset(this->buffer, 0xFF, sizeof(this->buffer));
    this->rollingBufferSilenceWindowScore = 0;
    this->rollingBufferCarrierWindowScore = MINUTE_MARKER_NUM_SAMPLES_CARRIER;
  }

  /// @brief Helper function that decodes a BCD value with given weights
  /// @param startIdx Start index of packed array to read bits from
  /// @param count  Number of bits to read
  /// @param weights Array of weights where each idx corresponds to value that bit contributes
  /// @return Decoded integer value
  int decodeBCD(int startIdx, int count, const int *weights)
  {
    int val = 0;
    for (int i = 0; i < count; i++)
    {
      if (startIdx + i >= 60)
        break;
      if (this->readBit(this->packedA, startIdx + i))
        val += weights[i];
    }
    return val;
  }

  /// @brief Helper function that checks parity against MSF spec
  /// @param startIdx Start index of packed array to read bits from
  /// @param count Number of bits to read
  /// @param parityBitIdx Index of parity bit in checksum array
  /// @return True if parity is correct, false otherwise
  bool checkParity(int startIdx, int count, int parityBitIdx)
  {
    int ones = 0;
    for (int i = 0; i < count; i++)
    {
      if (this->readBit(this->packedA, startIdx + i))
        ones++;
    }
    if (this->readBit(this->packedB, parityBitIdx))
      ones++;
    return (ones % 2 != 0);
  }

  /// @brief Helper function that sleeps for a random time between 1 and 5 seconds.
  // TODO: Add yield() if espressif platform
  void sleepForRandomTime()
  {
    uint32_t sleepTime = random(1000, 5000);

    MSF_LOG(F("[MSF] Sleeping for "));
    MSF_LOG(sleepTime);
    MSF_LOGLN(F("ms to avoid always syncing at the same time..."));

    uint32_t currentMillis = millis();
    while (millis() - currentMillis < sleepTime)
    {
      delay(1);
    }
  }

  /// @brief Function that searches for minute marker transition and returns the timestamp of when we think the minute marker transition happens.
  /// @return Timestamp in milliseconds of the detected minute marker transition
  uint32_t syncToMinuteMarker()
  {
    // This function continously reads the input via _reader function, passes the output to the updateRollingBuffer function, gets the score from its output
    // and keeps timestamp of when we see the best score. After this function we can setup our listening windows for reading A and B bits of MFS signal.

    // to avoid always syncing on the same spot if we are very close to the minute marker,
    // in case we miss it first time we ∏dont want to keep missing it
    this->sleepForRandomTime();

    MSF_LOGLN(F("[MSF] Syncing... (Scanning 65s for Minute Marker)"));

    // initialize the private variables used for tracking our rolling buffer and its score
    this->rollingBufferSetupAndCleanup();

    uint32_t startScan = millis();
    uint32_t lastSample = 0;
    uint32_t lastPrint = 0;
    int maxScoreSeen = 0;
    int lastCalculatedScore = 0;
    uint32_t timeOfMaxScore = 0;

    while (millis() - startScan < 65000)
    {
      uint32_t now = millis();
      if (now - lastSample >= SAMPLE_RATE_MS)
      {
        lastSample = now;
        // MSF spec defines presence of carrier as binary 0 and absence of carrier (silence) as binary 1
        // but we dont invert here because we are only intersted in carrier presence or absence
        int currentScore = this->updateRollingBuffer(this->_reader());
        lastCalculatedScore = currentScore;

        if (currentScore > maxScoreSeen)
        {
          maxScoreSeen = currentScore;
          timeOfMaxScore = now;
        }
      }

      if (now - lastPrint >= 100)
      {
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
    MSF_LOG(F("[MSF] Final Peak Score: "));
    MSF_LOG(maxScoreSeen);
    MSF_LOGLN();
    // we subtract 500ms because the silence window on minute marker ends 500ms after transition between carrier and silence,
    // but that transition actually marks the start of the minute
    return timeOfMaxScore - 500;
  }
  /// @brief Helper function that waits until the next minute boundary after syncing to the minute marker transition
  /// @return Timestamp in milliseconds of when we think the next minute starts, which is when we should start reading the bits of MSF signal
  uint32_t calculate_next_bit_retreival_window_start_timestamp()
  {
    uint32_t prevMinuteMillis = this->syncToMinuteMarker();

    // calculate how long we need to wait
    uint32_t elapsedSinceMarker = millis() - prevMinuteMillis;

    // The remainder tells us how far we are into the CURRENT 60s cycle.
    // Subtracting that from 60000 gives us the exact time remaining until the NEXT cycle.
    // this is needed as we are listening for more than 60s in syncToMinuteMarker function
    // so if we get result very early we cant just wait for hardcoded 60s, we might need more
    uint32_t waitInMiliseconds = 60000 - (elapsedSinceMarker % 60000);

    uint32_t nextMinuteMillis = millis() + waitInMiliseconds;

    return nextMinuteMillis;
  }

public:
  MSFReceiver(ReaderFunction readerFunc) : _reader(readerFunc) {}

  /// @brief Reads the MSF signal and outputs the decoded time and checksum result.
  /// @return Struct containing decoded time and checksum result.
  MSFData get_time()
  {

    uint32_t minuteStart = this->calculate_next_bit_retreival_window_start_timestamp();
    uint32_t nextSecondBoundary = 1000;
    uint32_t lastSample = 0;

    // Reset Member Variables
    memset(this->packedA, 0, sizeof(this->packedA));
    memset(this->packedB, 0, sizeof(this->packedB));

    MSF_LOG(F("[MSF] Sync Complete. Aligning to next minute (Will wait for: "));
    MSF_LOG((minuteStart - millis()));
    MSF_LOGLN(F("ms)..."));

    // 3. WAIT
    while (millis() < minuteStart)
    {
      delay(1);
    }
    MSF_LOGLN(F(" Starting decode NOW."));

    MSF_LOGLN(F("[MSF] ------------------------------------------------"));
    MSF_LOGLN(F("[MSF] SEC |   BIT A (135-165ms)   |   BIT B (235-265ms)"));
    MSF_LOGLN(F("[MSF] ------------------------------------------------"));

    int countOfHighBitAsamples = 0, totalCountOfBitAsamples = 0;
    int countOfHighBitBsamples = 0, totalCountOfBitBSamples = 0;
    int currentSecond = 0;

    while (currentSecond < 60)
    {
      uint32_t now = millis();
      uint32_t elapsed = now - minuteStart;

      if (now - lastSample >= SAMPLE_RATE_MS)
      {
        lastSample = now;
        int ms = elapsed % 1000;
        // MSF spec defines presence of carrier as binary 0 and absence of carrier (silence) as binary 1
        // we invert the carrier state here to make it more intuitive to work with, where 1 means presence of carrier and 0 means silence
        bool carrierState = this->_reader();
        bool binaryState = !carrierState;

        // Accumulate data if we are inside the specific windows for Bit A or Bit B
        // we read multiple time in the window to be more resilient and later we will take
        // vote based on percentage of samples
        if (ms >= 135 && ms <= 165)
        {
          totalCountOfBitAsamples++;
          if (binaryState)
            countOfHighBitAsamples++;
        }
        else if (ms >= 235 && ms <= 265)
        {
          totalCountOfBitBSamples++;
          if (binaryState)
            countOfHighBitBsamples++;
        }
      }

      // 2. PROCESS & STORE (End of Second)
      // Check if we crossed the 1000ms boundary. If so, calculate the final bit for the second.
      if (elapsed >= nextSecondBoundary)
      {
        int percentageOfHighAsamples = (totalCountOfBitAsamples > 0) ? (countOfHighBitAsamples * 100) / totalCountOfBitAsamples : 0;
        int percentageOfHighBitBsamples = (totalCountOfBitBSamples > 0) ? (countOfHighBitBsamples * 100) / totalCountOfBitBSamples : 0;

        bool valA = (percentageOfHighAsamples > 60);        // if more than 60% of the samples in bit A window are high, we consider the bit to be 1, otherwise 0
        bool valB = (percentageOfHighBitBsamples > 60);     // if more than 60% of the samples in bit B window are high, we consider the bit to be 1, otherwise 0
        this->writeBit(this->packedA, currentSecond, valA); // if more than 60% of the samples in bit A window are high, we consider the bit to be 1, otherwise 0
        this->writeBit(this->packedB, currentSecond, valB); // if more than 60% of the samples in bit B window are high, we consider the bit to be 1, otherwise 0

        MSF_LOG(F("[MSF] Sec "));
        if (currentSecond < 10)
          MSF_LOG(F("0"));
        MSF_LOG(currentSecond);
        MSF_LOG(F(" | A:"));
        MSF_LOG((valA) ? F("1") : F("0"));
        MSF_LOG(F(" ["));
        MSF_LOG(percentageOfHighAsamples);
        MSF_LOG(F("%]"));
        MSF_LOG(F(" | B:"));
        MSF_LOG((valB) ? F("1") : F("0"));
        MSF_LOG(F(" ["));
        MSF_LOG(percentageOfHighBitBsamples);
        MSF_LOG(F("%]"));

        if (percentageOfHighAsamples < 90 && percentageOfHighAsamples > 10)
          MSF_LOG(F(" <--- NOISY"));
        MSF_LOGLN();

        // Prepare for next second
        currentSecond++;
        nextSecondBoundary += 1000;
        countOfHighBitAsamples = 0;
        totalCountOfBitAsamples = 0;
        countOfHighBitBsamples = 0;
        totalCountOfBitBSamples = 0;
      }
    }
    MSF_LOGLN(F("[MSF] ------------------------------------------------"));

    // 3. DECODE
    MSFData result;
    result.time.Second = 0;
    result.time.Minute = 0;
    result.time.Hour = 0;
    result.time.Day = 1;
    result.time.Month = 1;
    result.time.Year = 0;

    static const int wYear[] = {80, 40, 20, 10, 8, 4, 2, 1};
    static const int wMonth[] = {10, 8, 4, 2, 1};
    static const int wDay[] = {20, 10, 8, 4, 2, 1};
    static const int wDOW[] = {4, 2, 1};
    static const int wHour[] = {20, 10, 8, 4, 2, 1};
    static const int wMin[] = {40, 20, 10, 8, 4, 2, 1};

    int rawYear = this->decodeBCD(17, 8, wYear);
    result.time.Year = rawYear + 30;
    result.time.Month = this->decodeBCD(25, 5, wMonth);
    result.time.Day = this->decodeBCD(30, 6, wDay);
    result.time.Hour = this->decodeBCD(39, 6, wHour);
    result.time.Minute = this->decodeBCD(45, 7, wMin);
    result.time.Wday = this->decodeBCD(36, 3, wDOW) + 1;

    // each piece of information has its own parity bit as in MSF spec
    bool pYear = this->checkParity(17, 8, 54);  // year is located from bit 17 to bit 24 in packedA, and its parity bit is located at bit 54 in packedB
    bool pDate = this->checkParity(25, 11, 55); // date (month, day, dow) is located from bit 25 to bit 35 in packedA, and its parity bit is located at bit 55 in packedB
    bool pDOW = this->checkParity(36, 3, 56);   // day of week is located from bit 36 to bit 38 in packedA, and its parity bit is located at bit 56 in packedB
    bool pTime = this->checkParity(39, 13, 57); // time (hour, minute) is located from bit 39 to bit 51 in packedA, and its parity bit is located at bit 57 in packedB

    bool sane = (result.time.Month >= 1 && result.time.Month <= 12) &&
                (result.time.Day >= 1 && result.time.Day <= 31) &&
                (result.time.Hour <= 23) && (result.time.Minute <= 59);

    result.checksum_passed = pYear && pDate && pDOW && pTime && sane;

    return result;
  }

  /// @brief Reads the MSF signal and outputs the decoded time and checksum result, with keep blocking and retrying until checksum is passed and we have a valid time.
  /// @return Struct containing decoded time and checksum result.
  MSFData get_time_with_retry()
  {
    while (true)
    {
      MSF_LOGLN(F("\n[MSF] Attempting to acquire atomic time..."));
      MSFData res = this->get_time();

      if (res.checksum_passed)
      {
        MSF_LOGLN(F("[MSF] SUCCESS! Checksum Passed."));
        return res;
      }
      else
      {
        MSF_LOGLN(F("[MSF] Checksum Failed. Retrying..."));
      }
    }
  }
};