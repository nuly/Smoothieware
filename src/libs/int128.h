#include <stdint.h>

class uint128_t {
    public:
        uint128_t();
        uint128_t(const uint64_t l);
        uint128_t(const uint64_t b, const uint64_t l);

        uint128_t operator+(const uint128_t& rhs) const;
        uint128_t operator+(const unsigned int& rhs) const;
        uint128_t operator-(const uint128_t& rhs) const;
        uint128_t operator-(const unsigned int& rhs) const;
        uint128_t operator*(const uint128_t& rhs) const;
        uint128_t operator*(const unsigned int& rhs) const;
        bool operator<(const uint128_t& rhs) const;
        bool operator>(const uint128_t& rhs) const;
        bool operator>=(const uint128_t& rhs) const;
        bool operator>=(const unsigned int& rhs) const;
        bool operator<(const unsigned int& rhs) const;
        bool operator<=(const unsigned int& rhs) const;
        bool operator<=(const uint128_t& rhs) const;
        void operator=(const uint128_t& rhs);
        void operator=(const unsigned int& rhs);

    private:
        uint64_t b;
        uint64_t l;
};
