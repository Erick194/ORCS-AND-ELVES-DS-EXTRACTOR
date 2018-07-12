
#ifndef Endian_h
#define Endian_h

#include "funciones.h"

inline word endianSwap16L(word x)
{
	return x;
}

inline word endianSwap16B(word x)
{
	return ((x >> 8) & 0x00ff) | ((x << 8) & 0xff00);
}

inline uint endianSwap32L(uint x)
{
	return x;
}

inline uint endianSwap32B(uint x)
{
	return (endianSwap16B(x & 0x000ffff) << 16) | (endianSwap16B(x >> 16));
}

inline byte endianReadU8Little(const byte* pos)
{
	return *pos;
}

inline byte endianReadU8Big(const byte* pos)
{
	return *pos;
}

inline word endianReadU16Little(const word* pos)
{
	return endianSwap16L(*pos);
}

inline word endianReadU16Big(const word* pos)
{
	return endianSwap16B(*pos);
}

inline uint endianReadU32Little(const uint* pos)
{
	return endianSwap32L(*pos);
}

inline uint endianReadU32Big(const uint* pos)
{
	return endianSwap32B(*pos);
}

inline void endianWriteU8Little(byte* pos, byte value)
{
	*pos = value;
}

inline void endianWriteU8Big(byte* pos, byte value)
{
	*pos = value;
}

inline void endianWriteU16Little(word* pos, word value)
{
	*pos = endianSwap16L(value);
}

inline void endianWriteU16Big(word* pos, word value)
{
	*pos = endianSwap16B(value);
}

inline void endianWriteU32Little(uint* pos, uint value)
{
	*pos = endianSwap32L(value);
}

inline void endianWriteU32Big(uint* pos, uint value)
{
	*pos = endianSwap32B(value);
}



#endif
