#include <chrono>

class Timer
{
public:
    Timer()
    {
        this->t0 = std::chrono::high_resolution_clock::now();
    }

    void start()
    {
        this->t0 = std::chrono::high_resolution_clock::now();
    }

    double getTime()
    {
        auto t = std::chrono::high_resolution_clock::now();
        auto d = std::chrono::duration_cast<std::chrono::microseconds>(t - this->t0);
        return d.count() / 1000.0;
    }

    void delay(double ms)
    {
        double t = this->getTime();
        while (this->getTime() - t < ms) {
        }
    }

private:
    std::chrono::high_resolution_clock::time_point t0;
};

inline Timer timer;