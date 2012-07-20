#ifndef __FILTERS_H
#define __FILTERS_H

#include <cstring>

namespace PTAMM {

template <typename ValueType, int N>
class MovingAverageFilter {
  public:
    MovingAverageFilter()
      : mFirstUpdate(true)
      , mValue(0)
      , mValueIdx(0)
    {
    }

    void Reset() {
      mFirstUpdate = true;
    }

    void Update(ValueType value)
    {
      if (mFirstUpdate) {
        mValue = value;
        value *= 1.0/N;
        for (size_t i = 0; i < N; ++i) {
          mOldValues[i] = value;
        }
        mFirstUpdate = false;
      } else {
        value *= 1.0/N;
        mValue += value - mOldValues[mValueIdx];
        mOldValues[mValueIdx] = value;
        mValueIdx = (mValueIdx + 1) % N;
      }
    }

    const ValueType& GetValue() const { return mValue; }

  private:
    bool mFirstUpdate;
    ValueType mValue;
    ValueType mOldValues[N];
    int mValueIdx;
};

}

#endif
