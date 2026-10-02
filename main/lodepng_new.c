/*
LodePNG version 20230410

Copyright (c) 2005-2023 Lode Vandevenne

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

    1. The origin of this software must not be misrepresented; you must not
    claim that you wrote the original software. If you use this software
    in a product, an acknowledgment in the product documentation would be
    appreciated but is not required.

    2. Altered source versions must be plainly marked as such, and must not be
    misrepresented as being the original software.

    3. This notice may not be removed or altered from any source
    distribution.
*/

/*
The manual and changelog are in the header file "lodepng.h"
Rename this file to lodepng.cpp to use it for C++, or to lodepng.c to use it for C.
*/

#include "lodepng.h"
#if LV_USE_LODEPNG
#include "../../core/lv_global.h"

#define image_cache_draw_buf_handlers &(LV_GLOBAL_DEFAULT()->image_cache_draw_buf_handlers)

#ifdef LODEPNG_COMPILE_DISK
    #include <limits.h> /* LONG_MAX */
    #include <stdio.h> /* file handling */
#endif /* LODEPNG_COMPILE_DISK */

#ifdef LODEPNG_COMPILE_ALLOCATORS
    #include <stdlib.h> /* allocations */
#endif /* LODEPNG_COMPILE_ALLOCATORS */

#if defined(_MSC_VER) && (_MSC_VER >= 1310) /*Visual Studio: A few warning types are not desired here.*/
    #pragma warning( disable : 4244 ) /*implicit conversions: not warned by gcc -Wall -Wextra and requires too much casts*/
    #pragma warning( disable : 4996 ) /*VS does not like fopen, but fopen_s is not standard C so unusable here*/
#endif /*_MSC_VER */

const char * LODEPNG_VERSION_STRING = "20230410";

/*
This source file is divided into the following large parts. The code sections
with the "LODEPNG_COMPILE_" #defines divide this up further in an intermixed way.
-Tools for C and common code for PNG and Zlib
-C Code for Zlib (huffman, deflate, ...)
-C Code for PNG (file format chunks, adam7, PNG filters, color conversions, ...)
-The C++ wrapper around all of the above
*/

/* ////////////////////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////////////////////// */
/* // Tools for C, and common code for PNG and Zlib.                       // */
/* ////////////////////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////////////////////// */

/*The malloc, realloc and free functions defined here with "lodepng_" in front
of the name, so that you can easily change them to others related to your
platform if needed. Everything else in the code calls these. Pass
-DLODEPNG_NO_COMPILE_ALLOCATORS to the compiler, or comment out
#define LODEPNG_COMPILE_ALLOCATORS in the header, to disable the ones here and
define them in your own project's source files without needing to change
lodepng source code. Don't forget to remove "static" if you copypaste them
from here.*/

#ifdef LODEPNG_COMPILE_ALLOCATORS
static void * lodepng_malloc(size_t size)
{
#ifdef LODEPNG_MAX_ALLOC
    if(size > LODEPNG_MAX_ALLOC) return 0;
#endif
    return lv_malloc(size);
}

/* NOTE: when realloc returns NULL, it leaves the original memory untouched */
static void * lodepng_realloc(void * ptr, size_t new_size)
{
#ifdef LODEPNG_MAX_ALLOC
    if(new_size > LODEPNG_MAX_ALLOC) return 0;
#endif
    return lv_realloc(ptr, new_size);
}

static void lodepng_free(void * ptr)
{
    lv_free(ptr);
}
#else /*LODEPNG_COMPILE_ALLOCATORS*/
/* TODO: support giving additional void* payload to the custom allocators */
void * lodepng_malloc(size_t size);
void * lodepng_realloc(void * ptr, size_t new_size);
void lodepng_free(void * ptr);
#endif /*LODEPNG_COMPILE_ALLOCATORS*/

/* convince the compiler to inline a function, for use when this measurably improves performance */
/* inline is not available in C90, but use it when supported by the compiler */
#if (defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901L)) || (defined(__cplusplus) && (__cplusplus >= 199711L))
    #define LODEPNG_INLINE inline
#else
    #define LODEPNG_INLINE /* not available */
#endif

/* restrict is not available in C90, but use it when supported by the compiler */
#if (defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1))) ||\
    (defined(_MSC_VER) && (_MSC_VER >= 1400)) || \
    (defined(__WATCOMC__) && (__WATCOMC__ >= 1250) && !defined(__cplusplus))
    #define LODEPNG_RESTRICT __restrict
#else
    #define LODEPNG_RESTRICT /* not available */
#endif

/* Replacements for C library functions such as memcpy and strlen, to support platforms
where a full C library is not available. The compiler can recognize them and compile
to something as fast. */

static void lodepng_memcpy(void * LODEPNG_RESTRICT dst,
                           const void * LODEPNG_RESTRICT src, size_t size)
{
    lv_memcpy(dst, src, size);
}

static void lodepng_memset(void * LODEPNG_RESTRICT dst,
                           int value, size_t num)
{
    lv_memset(dst, value, num);
}

/* does not check memory out of bounds, do not use on untrusted data */
static size_t lodepng_strlen(const char * a)
{
    const char * orig = a;
    /* avoid warning about unused function in case of disabled COMPILE... macros */
    (void)(&lodepng_strlen);
    while(*a) a++;
    return (size_t)(a - orig);
}

#define LODEPNG_MAX(a, b) (((a) > (b)) ? (a) : (b))
#define LODEPNG_MIN(a, b) (((a) < (b)) ? (a) : (b))

#if defined(LODEPNG_COMPILE_PNG) || defined(LODEPNG_COMPILE_DECODER)
/* Safely check if adding two integers will overflow (no undefined
behavior, compiler removing the code, etc...) and output result. */
static int lodepng_addofl(size_t a, size_t b, size_t * result)
{
    *result = a + b; /* Unsigned addition is well defined and safe in C90 */
    return *result < a;
}
#endif /*defined(LODEPNG_COMPILE_PNG) || defined(LODEPNG_COMPILE_DECODER)*/

#ifdef LODEPNG_COMPILE_DECODER
/* Safely check if multiplying two integers will overflow (no undefined
behavior, compiler removing the code, etc...) and output result. */
static int lodepng_mulofl(size_t a, size_t b, size_t * result)
{
    *result = a * b; /* Unsigned multiplication is well defined and safe in C90 */
    return (a != 0 && *result / a != b);
}

#ifdef LODEPNG_COMPILE_ZLIB
/* Safely check if a + b > c, even if overflow could happen. */
static int lodepng_gtofl(size_t a, size_t b, size_t c)
{
    size_t d;
    if(lodepng_addofl(a, b, &d)) return 1;
    return d > c;
}
#endif /*LODEPNG_COMPILE_ZLIB*/
#endif /*LODEPNG_COMPILE_DECODER*/


/*
Often in case of an error a value is assigned to a variable and then it breaks
out of a loop (to go to the cleanup phase of a function). This macro does that.
It makes the error handling code shorter and more readable.

Example: if(!uivector_resize(&lz77_encoded, datasize)) ERROR_BREAK(83);
*/
#define CERROR_BREAK(errorvar, code){\
        errorvar = code;\
        break;\
    }

/*version of CERROR_BREAK that assumes the common case where the error variable is named "error"*/
#define ERROR_BREAK(code) CERROR_BREAK(error, code)

/*Set error var to the error code, and return it.*/
#define CERROR_RETURN_ERROR(errorvar, code){\
        errorvar = code;\
        return code;\
    }

/*Try the code, if it returns error, also return the error.*/
#define CERROR_TRY_RETURN(call){\
        unsigned error = call;\
        if(error) return error;\
    }

/*Set error var to the error code, and return from the void function.*/
#define CERROR_RETURN(errorvar, code){\
        errorvar = code;\
        return;\
    }

/*
About uivector, ucvector and string:
-All of them wrap dynamic arrays or text strings in a similar way.
-LodePNG was originally written in C++. The vectors replace the std::vectors that were used in the C++ version.
-The string tools are made to avoid problems with compilers that declare things like strncat as deprecated.
-They're not used in the interface, only internally in this file as static functions.
-As with many other structs in this file, the init and cleanup functions serve as ctor and dtor.
*/

#ifdef LODEPNG_COMPILE_ZLIB
#ifdef LODEPNG_COMPILE_ENCODER
/*dynamic vector of unsigned ints*/
typedef struct uivector {
    unsigned * data;
    size_t size; /*size in number of unsigned longs*/
    size_t allocsize; /*allocated size in bytes*/
} uivector;

static void uivector_cleanup(void * p)
{
    ((uivector *)p)->size = ((uivector *)p)->allocsize = 0;
    lodepng_free(((uivector *)p)->data);
    ((uivector *)p)->data = NULL;
}

/*returns 1 if success, 0 if failure ==> nothing done*/
static unsigned uivector_resize(uivector * p, size_t size)
{
    size_t allocsize = size * sizeof(unsigned);
    if(allocsize > p->allocsize) {
        size_t newsize = allocsize + (p->allocsize >> 1u);
        void * data = lodepng_realloc(p->data, newsize);
        if(data) {
            p->allocsize = newsize;
            p->data = (unsigned *)data;
        }
        else return 0; /*error: not enough memory*/
    }
    p->size = size;
    return 1; /*success*/
}

static void uivector_init(uivector * p)
{
    p->data = NULL;
    p->size = p->allocsize = 0;
}

/*returns 1 if success, 0 if failure ==> nothing done*/
static unsigned uivector_push_back(uivector * p, unsigned c)
{
    if(!uivector_resize(p, p->size + 1)) return 0;
    p->data[p->size - 1] = c;
    return 1;
}
#endif /*LODEPNG_COMPILE_ENCODER*/
#endif /*LODEPNG_COMPILE_ZLIB*/

/* /////////////////////////////////////////////////////////////////////////// */

/*dynamic vector of unsigned chars*/
typedef struct ucvector {
    unsigned char * data;
    size_t size; /*used size*/
    size_t allocsize; /*allocated size*/
} ucvector;

/*returns 1 if success, 0 if failure ==> nothing done*/
static unsigned ucvector_reserve(ucvector * p, size_t size)
{
    if(size > p->allocsize) {
        size_t newsize = size + (p->allocsize >> 1u);
        void * data = lodepng_realloc(p->data, newsize);
        if(data) {
            p->allocsize = newsize;
            p->data = (unsigned char *)data;
        }
        else return 0; /*error: not enough memory*/
    }
    return 1; /*success*/
}

/*returns 1 if success, 0 if failure ==> nothing done*/
static unsigned ucvector_resize(ucvector * p, size_t size)
{
    p->size = size;
    return ucvector_reserve(p, size);
}

static ucvector ucvector_init(unsigned char * buffer, size_t size)
{
    ucvector v;
    v.data = buffer;
    v.allocsize = v.size = size;
    return v;
}

/* ////////////////////////////////////////////////////////////////////////// */

#ifdef LODEPNG_COMPILE_PNG
#ifdef LODEPNG_COMPILE_ANCILLARY_CHUNKS

/*free string pointer and set it to NULL*/
static void string_cleanup(char ** out)
{
    lodepng_free(*out);
    *out = NULL;
}

/*also appends null termination character*/
static char * alloc_string_sized(const char * in, size_t insize)
{
    char * out = (char *)lodepng_malloc(insize + 1);
    if(out) {
        lodepng_memcpy(out, in, insize);
        out[insize] = 0;
    }
    return out;
}

/* dynamically allocates a new string with a copy of the null terminated input text */
static char * alloc_string(const char * in)
{
    return alloc_string_sized(in, lodepng_strlen(in));
}
#endif /*LODEPNG_COMPILE_ANCILLARY_CHUNKS*/
#endif /*LODEPNG_COMPILE_PNG*/

/* ////////////////////////////////////////////////////////////////////////// */

#if defined(LODEPNG_COMPILE_DECODER) || defined(LODEPNG_COMPILE_PNG)
static unsigned lodepng_read32bitInt(const unsigned char * buffer)
{
    return (((unsigned)buffer[0] << 24u) | ((unsigned)buffer[1] << 16u) |
            ((unsigned)buffer[2] << 8u) | (unsigned)buffer[3]);
}
#endif /*defined(LODEPNG_COMPILE_DECODER) || defined(LODEPNG_COMPILE_PNG)*/

#if defined(LODEPNG_COMPILE_PNG) || defined(LODEPNG_COMPILE_ENCODER)
/*buffer must have at least 4 allocated bytes available*/
static void lodepng_set32bitInt(unsigned char * buffer, unsigned value)
{
    buffer[0] = (unsigned char)((value >> 24) & 0xff);
    buffer[1] = (unsigned char)((value >> 16) & 0xff);
    buffer[2] = (unsigned char)((value >>  8) & 0xff);
    buffer[3] = (unsigned char)((value) & 0xff);
}
#endif /*defined(LODEPNG_COMPILE_PNG) || defined(LODEPNG_COMPILE_ENCODER)*/

/* ////////////////////////////////////////////////////////////////////////// */
/* / File IO                                                                / */
/* ////////////////////////////////////////////////////////////////////////// */

#ifdef LODEPNG_COMPILE_DISK

/* returns negative value on error. This should be pure C compatible, so no fstat. */
static long lodepng_filesize(const char * filename)
{
    lv_fs_file_t f;
    lv_fs_res_t res = lv_fs_open(&f, filename, LV_FS_MODE_RD);
    if(res != LV_FS_RES_OK) return -1;
    uint32_t size = 0;
    if(lv_fs_seek(&f, 0, LV_FS_SEEK_END) != 0) {
        lv_fs_close(&f);
        return -1;
    }

    lv_fs_tell(&f, &size);
    lv_fs_close(&f);
    return size;
}

/* load file into buffer that already has the correct allocated size. Returns error code.*/
static unsigned lodepng_buffer_file(unsigned char * out, size_t size, const char * filename)
{
    lv_fs_file_t f;
    lv_fs_res_t res = lv_fs_open(&f, filename, LV_FS_MODE_RD);
    if(res != LV_FS_RES_OK) return 78;

    uint32_t br;
    res = lv_fs_read(&f, out, size, &br);
    lv_fs_close(&f);

    if(res != LV_FS_RES_OK) return 78;
    if(br != size) return 78;

    return 0;
}

unsigned lodepng_load_file(unsigned char ** out, size_t * outsize, const char * filename)
{
    long size = lodepng_filesize(filename);
    if(size < 0) return 78;
    *outsize = (size_t)size;

    *out = (unsigned char *)lodepng_malloc((size_t)size);
    if(!(*out) && size > 0) return 83; /*the above malloc failed*/

    return lodepng_buffer_file(*out, (size_t)size, filename);
}

/*write given buffer to the file, overwriting the file, it doesn't append to it.*/
unsigned lodepng_save_file(const unsigned char * buffer, size_t buffersize, const char * filename)
{
    lv_fs_file_t f;
    lv_fs_res_t res = lv_fs_open(&f, filename, LV_FS_MODE_WR);
    if(res != LV_FS_RES_OK) return 79;

    uint32_t bw;
    res = lv_fs_write(&f, buffer, buffersize, &bw);
    lv_fs_close(&f);
    return 0;
}

#endif /*LODEPNG_COMPILE_DISK*/

/* ////////////////////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////////////////////// */
/* // End of common code and tools. Begin of Zlib related code.            // */
/* ////////////////////////////////////////////////////////////////////////// */
/* ////////////////////////////////////////////////////////////////////////// */

#ifdef LODEPNG_COMPILE_ZLIB
#ifdef LODEPNG_COMPILE_ENCODER

typedef struct {
    ucvector * data;
    unsigned char bp; /*ok to overflow, indicates bit pos inside byte*/
} LodePNGBitWriter;

static void LodePNGBitWriter_init(LodePNGBitWriter * writer, ucvector * data)
{
    writer->data = data;
    writer->bp = 0;
}

/*TODO: this ignores potential out of memory errors*/
#define WRITEBIT(writer, bit){\
        /* append new byte */\
        if(((writer->bp) & 7u) == 0) {\
            if(!ucvector_resize(writer->data, writer->data->size + 1)) return;\
            writer->data->data[writer->data->size - 1] = 0;\
        }\
        (writer->data->data[writer->data->size - 1]) |= (bit << ((writer->bp) & 7u));\
        ++writer->bp;\
    }

/* LSB of value is written first, and LSB of bytes is used first */
static void writeBits(LodePNGBitWriter * writer, unsigned value, size_t nbits)
{
    if(nbits == 1) { /* compiler should statically compile this case if nbits == 1 */
        WRITEBIT(writer, value);
    }
    else {
        /* TODO: increase output size only once here rather than in each WRITEBIT */
        size_t i;
        for(i = 0; i != nbits; ++i) {
            WRITEBIT(writer, (unsigned char)((value >> i) & 1));
        }
    }
}

/* This one is to use for adding huffman symbol, the value bits are written MSB first */
static void writeBitsReversed(LodePNGBitWriter * writer, unsigned value, size_t nbits)
{
    size_t i;
    for(i = 0; i != nbits; ++i) {
        /* TODO: increase output size only once here rather than in each WRITEBIT */
        WRITEBIT(writer, (unsigned char)((value >> (nbits - 1u - i)) & 1u));
    }
}
#endif /*LODEPNG_COMPILE_ENCODER*/

#ifdef LODEPNG_COMPILE_DECODER

typedef struct {
    const unsigned char * data;
    size_t size; /*size of data in bytes*/
    size_t bitsize; /*size of data in bits, end of valid bp values, should be 8*size*/
    size_t bp;
    unsigned buffer; /*buffer for reading bits. NOTE: 'unsigned' must support at least 32 bits*/
} LodePNGBitReader;

/* data size argument is in bytes. Returns error if size too large causing overflow */
static unsigned LodePNGBitReader_init(LodePNGBitReader * reader, const unsigned char * data, size_t size)
{
    size_t temp;
    reader->data = data;
    reader->size = size;
    /* size in bits, return error if overflow (if size_t is 32 bit this supports up to 500MB)  */
    if(lodepng_mulofl(size, 8u, &reader->bitsize)) return 105;
    /*ensure incremented bp can be compared to bitsize without overflow even when it would be incremented 32 too much and
    trying to ensure 32 more bits*/
    if(lodepng_addofl(reader->bitsize, 64u, &temp)) return 105;
    reader->bp = 0;
    reader->buffer = 0;
    return 0; /*ok*/
}

/*
ensureBits functions:
Ensures the reader can at least read nbits bits in one or more readBits calls,
safely even if not enough bits are available.
The nbits parameter is unused but is given for documentation purposes, error
checking for amount of bits must be done beforehand.
*/

/*See ensureBits documentation above. This one ensures up to 9 bits */
static LODEPNG_INLINE void ensureBits9(LodePNGBitReader * reader, size_t nbits)
{
    size_t start = reader->bp >> 3u;
    size_t size = reader->size;
    if(start + 1u < size) {
        reader->buffer = (unsigned)reader->data[start + 0] | ((unsigned)reader->data[start + 1] << 8u);
        reader->buffer >>= (reader->bp & 7u);
    }
    else {
        reader->buffer = 0;
        if(start + 0u < size) reader->buffer = reader->data[start + 0];
        reader->buffer >>= (reader->bp & 7u);
    }
    (void)nbits;
}

/*See ensureBits documentation above. This one ensures up to 17 bits */
static LODEPNG_INLINE void ensureBits17(LodePNGBitReader * reader, size_t nbits)
{
    size_t start = reader->bp >> 3u;
    size_t size = reader->size;
    if(start + 2u < size) {
        reader->buffer = (unsigned)reader->data[start + 0] | ((unsigned)reader->data[start + 1] << 8u) |
                         ((unsigned)reader->data[start + 2] << 16u);
        reader->buffer >>= (reader->bp & 7u);
    }
    else {
        reader->buffer = 0;
        if(start + 0u < size) reader->buffer |= reader->data[start + 0];
        if(start + 1u < size) reader->buffer |= ((unsigned)reader->data[start + 1] << 8u);
        reader->buffer >>= (reader->bp & 7u);
    }
    (void)nbits;
}

/*See ensureBits documentation above. This one ensures up to 25 bits */
static LODEPNG_INLINE void ensureBits25(LodePNGBitReader * reader, size_t nbits)
{
    size_t start = reader->bp >> 3u;
    size_t size = reader->size;
    if(start + 3u < size) {
        reader->buffer = (unsigned)reader->data[start + 0] | ((unsigned)reader->data[start + 1] << 8u) |
                         ((unsigned)reader->data[start + 2] << 16u) | ((unsigned)reader->data[start + 3] << 24u);
        reader->buffer >>= (reader->bp & 7u);
    }
    else {
        reader->buffer = 0;
        if(start + 0u < size) reader->buffer |= reader->data[start + 0];
        if(start + 1u < size) reader->buffer |= ((unsigned)reader->data[start + 1] << 8u);
        if(start + 2u < size) reader->buffer |= ((unsigned)reader->data[start + 2] << 16u);
        reader->buffer >>= (reader->bp & 7u);
    }
    (void)nbits;
}

/*See ensureBits documentation above. This one ensures up to 32 bits */
static LODEPNG_INLINE void ensureBits32(LodePNGBitReader * reader, size_t nbits)
{
    size_t start = reader->bp >> 3u;
    size_t size = reader->size;
    if(start + 4u < size) {
        reader->buffer = (unsigned)reader->data[start + 0] | ((unsigned)reader->data[start + 1] << 8u) |
                         ((unsigned)reader->data[start + 2] << 16u) | ((unsigned)reader->data[start + 3] << 24u);
        reader->buffer >>= (reader->bp & 7u);
        reader->buffer |= (((unsigned)reader->data[start + 4] << 24u) << (8u - (reader->bp & 7u)));
    }
    else {
        reader->buffer = 0;
        if(start + 0u < size) reader->buffer |= reader->data[start + 0];
        if(start + 1u < size) reader->buffer |= ((unsigned)reader->data[start + 1] << 8u);
        if(start + 2u < size) reader->buffer |= ((unsigned)reader->data[start + 2] << 16u);
        if(start + 3u < size) reader->buffer |= ((unsigned)reader->data[start + 3] << 24u);
        reader->buffer >>= (reader->bp & 7u);
    }
    (void)nbits;
}

/* Get bits without advancing the bit pointer. Must have enough bits available with ensureBits. Max nbits is 31. */
static LODEPNG_INLINE unsigned peekBits(LodePNGBitReader * reader, size_t nbits)
{
    /* The shift allows nbits to be only up to 31. */
    return reader->buffer & ((1u << nbits) - 1u);
}

/* Must have enough bits available with ensureBits */
static LODEPNG_INLINE void advanceBits(LodePNGBitReader * reader, size_t nbits)
{
    reader->buffer >>= nbits;
    reader->bp += nbits;
}

/* Must have enough bits available with ensureBits */
static LODEPNG_INLINE unsigned readBits(LodePNGBitReader * reader, size_t nbits)
{
    unsigned result = peekBits(reader, nbits);
    advanceBits(reader, nbits);
    return result;
}
#endif /*LODEPNG_COMPILE_DECODER*/

static unsigned reverseBits(unsigned bits, unsigned num)
{
    /*TODO: implement faster lookup table based version when needed*/
    unsigned i, result = 0;
    for(i = 0; i < num; i++) result |= ((bits >> (num - i - 1u)) & 1u) << i;
    return result;
}

/* ////////////////////////////////////////////////////////////////////////// */
/* / Deflate - Huffman                                                      / */
/* ////////////////////////////////////////////////////////////////////////// */

#define FIRST_LENGTH_CODE_INDEX 257
#define LAST_LENGTH_CODE_INDEX 285
/*256 literals, the end code, some length codes, and 2 unused codes*/
#define NUM_DEFLATE_CODE_SYMBOLS 288
/*the distance codes have their own symbols, 30 used, 2 unused*/
#define NUM_DISTANCE_SYMBOLS 32
/*the code length codes. 0-15: code lengths, 16: copy previous 3-6 times, 17: 3-10 zeros, 18: 11-138 zeros*/
#define NUM_CODE_LENGTH_CODES 19

/*the base lengths represented by codes 257-285*/
static const unsigned LENGTHBASE[29]
    = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59,
       67, 83, 99, 115, 131, 163, 195, 227, 258
      };

/*the extra bits used by codes 257-285 (added to base length)*/
static const unsigned LENGTHEXTRA[29]
    = {0, 0, 0, 0, 0, 0, 0,  0,  1,  1,  1,  1,  2,  2,  2,  2,  3,  3,  3,  3,
       4,  4,  4,   4,   5,   5,   5,   5,   0
      };

/*the base backwards distances (the bits of distance codes appear after length codes and use their own huffman tree)*/
static const unsigned DISTANCEBASE[30]
    = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513,
       769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577
      };

/*the extra bits of backwards distances (added to base)*/
static const unsigned DISTANCEEXTRA[30]
    = {0, 0, 0, 0, 1, 1, 2,  2,  3,  3,  4,  4,  5,  5,   6,   6,   7,   7,   8,
       8,    9,    9,   10,   10,   11,   11,   12,    12,    13,    13
      };

/*the order in which "code length alphabet code lengths" are stored as specified by deflate, out of this the huffman
tree of the dynamic huffman tree lengths is generated*/
static const unsigned CLCL_ORDER[NUM_CODE_LENGTH_CODES]
    = {16, 17, 18, 0, 8, 7, 9, 6, 10, 5, 11, 4, 12, 3, 13, 2, 14, 1, 15};

/* ////////////////////////////////////////////////////////////////////////// */

/*
Huffman tree struct, containing multiple representations of the tree
*/
typedef struct HuffmanTree {
    unsigned * codes; /*the huffman codes (bit patterns representing the symbols)*/
    unsigned * lengths; /*the lengths of the huffman codes*/
    unsigned maxbitlen; /*maximum number of bits a single code can get*/
    unsigned numcodes; /*number of symbols in the alphabet = number of codes*/
    /* for reading only */
    unsigned char * table_len; /*length of symbol from lookup table, or max length if secondary lookup needed*/
    unsigned short * table_value; /*value of symbol from lookup table, or pointer to secondary table if needed*/
} HuffmanTree;

static void HuffmanTree_init(HuffmanTree * tree)
{
    tree->codes = 0;
    tree->lengths = 0;
    tree->table_len = 0;
    tree->table_value = 0;
}

static void HuffmanTree_cleanup(HuffmanTree * tree)
{
    lodepng_free(tree->codes);
    lodepng_free(tree->lengths);
    lodepng_free(tree->table_len);
    lodepng_free(tree->table_value);
}

/* amount of bits for first huffman table lookup (aka root bits), see HuffmanTree_makeTable and huffmanDecodeSymbol.*/
/* values 8u and 9u work the fastest */
#define FIRSTBITS 9u

/* a symbol value too big to represent any valid symbol, to indicate reading disallowed huffman bits combination,
which is possible in case of only 0 or 1 present symbols. */
#define INVALIDSYMBOL 65535u

/* make table for huffman decoding */
static unsigned HuffmanTree_makeTable(HuffmanTree * tree)
{
    static const unsigned headsize = 1u << FIRSTBITS; /*size of the first table*/
    static const unsigned mask = (1u << FIRSTBITS) /*headsize*/ - 1u;
    size_t i, numpresent, pointer, size; /*total table size*/
    unsigned * maxlens = (unsigned *)lodepng_malloc(headsize * sizeof(unsigned));
    if(!maxlens) return 83; /*alloc fail*/

    /* compute maxlens: max total bit length of symbols sharing prefix in the first table*/
    lodepng_memset(maxlens, 0, headsize * sizeof(*maxlens));
    for(i = 0; i < tree->numcodes; i++) {
        unsigned symbol = tree->codes[i];
        unsigned l = tree->lengths[i];
        unsigned index;
        if(l <= FIRSTBITS) continue; /*symbols that fit in first table don't increase secondary table size*/
        /*get the FIRSTBITS MSBs, the MSBs of the symbol are encoded first. See later comment about the reversing*/
        index = reverseBits(symbol >> (l - FIRSTBITS), FIRSTBITS);
        maxlens[index] = LODEPNG_MAX(maxlens[index], l);
    }
    /* compute total table size: size of first table plus all secondary tables for symbols longer than FIRSTBITS */
    size = headsize;
    for(i = 0; i < headsize; ++i) {
        unsigned l = maxlens[i];
        if(l > FIRSTBITS) size += (((size_t)1) << (l - FIRSTBITS));
    }
    tree->table_len = (unsigned char *)lodepng_malloc(size * sizeof(*tree->table_len));
    tree->table_value = (unsigned short *)lodepng_malloc(size * sizeof(*tree->table_value));
    if(!tree->table_len || !tree->table_value) {
        lodepng_free(maxlens);
        /* freeing tree->table values is done at a higher scope */
        return 83; /*alloc fail*/
    }
    /*initialize with an invalid length to indicate unused entries*/
    for(i = 0; i < size; ++i) tree->table_len[i] = 16;

    /*fill in the first table for long symbols: max prefix size and pointer to secondary tables*/
    pointer = headsize;
    for(i = 0; i < headsize; ++i) {
        unsigned l = maxlens[i];
        if(l <= FIRSTBITS) continue;
        tree->table_len[i] = l;
        tree->table_value[i] = (unsigned short)pointer;
        pointer += (((size_t)1) << (l - FIRSTBITS));
    }
    lodepng_free(maxlens);

    /*fill in the first table for short symbols, or secondary table for long symbols*/
    numpresent = 0;
    for(i = 0; i < tree->numcodes; ++i) {
        unsigned l = tree->lengths[i];
        unsigned symbol, reverse;
        if(l == 0) continue;
        symbol = tree->codes[i]; /*the huffman bit pattern. i itself is the value.*/
        /*reverse bits, because the huffman bits are given in MSB first order but the bit reader reads LSB first*/
        reverse = reverseBits(symbol, l);
        numpresent++;

        if(l <= FIRSTBITS) {
            /*short symbol, fully in first table, replicated num times if l < FIRSTBITS*/
            unsigned num = 1u << (FIRSTBITS - l);
            unsigned j;
            for(j = 0; j < num; ++j) {
                /*bit reader will read the l bits of symbol first, the remaining FIRSTBITS - l bits go to the MSB's*/
                unsigned index = reverse | (j << l);
                if(tree->table_len[index] != 16) return 55; /*invalid tree: long symbol shares prefix with short symbol*/
                tree->table_len[index] = l;
                tree->table_value[index] = (unsigned short)i;
            }
        }
        else {
            /*long symbol, shares prefix with other long symbols in first lookup table, needs second lookup*/
            /*the FIRSTBITS MSBs of the symbol are the first table index*/
            unsigned index = reverse & mask;
            unsigned maxlen = tree->table_len[index];
            /*log2 of secondary table length, should be >= l - FIRSTBITS*/
            unsigned tablelen = maxlen - FIRSTBITS;
            unsigned start = tree->table_value[index]; /*starting index in secondary table*/
            unsigned num = 1u << (tablelen - (l - FIRSTBITS)); /*amount of entries of this symbol in secondary table*/
            unsigned j;
            if(maxlen < l) return 55; /*invalid tree: long symbol shares prefix with short symbol*/
            for(j = 0; j < num; ++j) {
                unsigned reverse2 = reverse >> FIRSTBITS; /* l - FIRSTBITS bits */
                unsigned index2 = start + (reverse2 | (j << (l - FIRSTBITS)));
                tree->table_len[index2] = l;
                tree->table_value[index2] = (unsigned short)i;
            }
        }
    }

    if(numpresent < 2) {
        /* In case of exactly 1 symbol, in theory the huffman symbol needs 0 bits,
        but deflate uses 1 bit instead. In case of 0 symbols, no symbols can
        appear at all, but such huffman tree could still exist (e.g. if distance
        codes are never used). In both cases, not all symbols of the table will be
        filled in. Fill them in with an invalid symbol value so returning them from
        huffmanDecodeSymbol will cause error. */
        for(i = 0; i < size; ++i) {
            if(tree->table_len[i] == 16) {
                /* As length, use a value smaller than FIRSTBITS for the head table,
                and a value larger than FIRSTBITS for the secondary table, to ensure
                valid behavior for advanceBits when reading this symbol. */
                tree->table_len[i] = (i < headsize) ? 1 : (FIRSTBITS + 1);
                tree->table_value[i] = INVALIDSYMBOL;
            }
        }
    }
    else {
        /* A good h