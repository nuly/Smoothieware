#include "int128.h"

#define LOHALF  0x00000000FFFFFFFF
#define QWIDTH  32

uint128_t::uint128_t() {
    this->b = 0;
    this->l = 0;
}
uint128_t::uint128_t(const uint64_t l) {
    this->b = 0;
    this->l = l;
}
uint128_t::uint128_t(const uint64_t b, const uint64_t l) {
    this->b = b;
    this->l = l;
}

bool
uint128_t::operator>=(const unsigned int& b) const {
    return (this->b > 0) || (this->l > b);
}

bool
uint128_t::operator<(const unsigned int& b) const {
    return (this->b == 0) && (this->l < b);
}

bool
uint128_t::operator<=(const unsigned int& b) const {
    return (this->b == 0) && (this->l <= b);
}

bool
uint128_t::operator<(const uint128_t& b) const {
    return (this->b < b.b) || (this->b == b.b && this->l < b.l);
}

bool
uint128_t::operator>(const uint128_t& b) const {
    return (this->b > b.b) || (this->b == b.b && this->l > b.l);
}

bool
uint128_t::operator<=(const uint128_t& b) const {
    return (this->b < b.b) || (this->b == b.b && this->l <= b.l);
}

bool
uint128_t::operator>=(const uint128_t& b) const {
    return (this->b > b.b) || (this->b == b.b && this->l >= b.l);
}

uint128_t
uint128_t::operator+(const unsigned int& b) const {
    uint128_t rval;
    rval.l = this->l + b;
    rval.b = this->b + (this->l > UINT64_MAX - b) ? 1 : 0;
    return rval;
}

uint128_t
uint128_t::operator+(const uint128_t& b) const {
    uint128_t rval;
    rval.l = this->l + b.l;
    rval.b = this->b + b.b + (this->l > UINT64_MAX - b.l) ? 1 : 0;
    return rval;
}

uint128_t
uint128_t::operator-(const uint128_t& b) const {
    uint128_t rval;
    if (this->b > b.b || (this->b == b.b && this->l > b.l)) {
        rval.b = this->b - b.b;
        if (this->l >= b.l) {
            rval.l = this->l - b.l;
        } else {
            rval.l = UINT64_MAX - (b.l - this->l + 1);
        }
        return rval;
    } else {
        return b - (*this);
    }
}

uint128_t
uint128_t::operator-(const unsigned int& b) const {
    uint128_t rval;
    if (this->l >= b) {
        rval.l = this->l - b;
        rval.b = this->b;
    } else {
        rval.l = UINT64_MAX - (b - this->l + 1);
        rval.b = this->b - 1;
    }
    return rval;
}

uint128_t
uint128_t::operator*(const uint128_t& b) const {
    uint128_t rval;

    uint64_t add1 = (this->l & LOHALF) * (b.l & LOHALF);
    uint64_t add2 = (this->l & LOHALF) * (b.l >> QWIDTH);
    uint64_t add3 = (this->l >> QWIDTH) * (b.l & LOHALF);
    uint64_t add4 = (this->l >> QWIDTH) * (b.l >> QWIDTH);

    rval.l = add1 + (add2 << QWIDTH);
    rval.b = (add1 > UINT64_MAX - (add2 << QWIDTH)) ? 1 : 0
        + (rval.l > UINT64_MAX - (add3 << QWIDTH)) ? 1 : 0;
    rval.l += (add3 << QWIDTH);
    rval.b += (add2 >> QWIDTH) + (add3 >> QWIDTH) + add4 + this->l * b.b + this->b * b.l;

    // if this->b and b.b are both positive, OVERFLOW

    return rval;
}

uint128_t
uint128_t::operator*(const unsigned int& b) const {
    return (*this)*uint128_t(0, b);
}

void
uint128_t::operator=(const uint128_t& rhs) {
    this->l = rhs.l;
    this->b = rhs.b;
}

void
uint128_t::operator=(const unsigned int& rhs) {
    this->l = rhs;
    this->b = 0;
}
