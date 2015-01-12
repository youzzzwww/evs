/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/** \file jbm_jb4_circularbuffer.c circular buffer (FIFO) with fixed capacity */

/* system includes */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
/* local includes */
#include "jbm_jb4_circularbuffer.h"
/* instrumentation */
#include "stl.h"
#include "options.h"
#include "basop_util.h"


/** Calculates percentile by selecting greatest elements.
 * This function partial sorts all given elements in the given buffer.
 * @param[in,out] elements ascending sorted buffer of selected greatest elements
 * @param[in,out] size size of elements buffer
 * @param[in]     capacity maximum number of elements to buffer
 * @param[in]     newElement element to insert in buffer if great enough */
static void JB4_CIRCULARBUFFER_calcPercentile( JB4_CIRCULARBUFFER_ELEMENT *elements,
        Word32 *size, Word32 capacity, JB4_CIRCULARBUFFER_ELEMENT newElement );


/** circular buffer (FIFO) with fixed capacity */
struct JB4_CIRCULARBUFFER
{
    /** elements of circular buffer */
    JB4_CIRCULARBUFFER_ELEMENT *data;
    /** maximum allowed number of elements plus one free element (to decide between full/empty buffer) */
    Word16 capacity;
    /** position of next enque operation */
    Word16 writePos;
    /** position of next deque operation */
    Word16 readPos;
};


/* Creates a circular buffer (FIFO) */
Word16 JB4_CIRCULARBUFFER_Create( JB4_CIRCULARBUFFER_HANDLE *ph )
{
    JB4_CIRCULARBUFFER_HANDLE h = malloc( sizeof( struct JB4_CIRCULARBUFFER ) );

    h->data     = NULL;
    move16();
    h->capacity = 0;
    move16();
    h->writePos = 0;
    move16();
    h->readPos  = 0;
    move16();

    *ph = h;
    move16();
    return 0;
}

/* Destroys the circular buffer (FIFO) */
void JB4_CIRCULARBUFFER_Destroy( JB4_CIRCULARBUFFER_HANDLE *ph )
{
    JB4_CIRCULARBUFFER_HANDLE h;

    IF( !ph )
    {
        return;
    }
    h = *ph;
    move16();
    IF( !h )
    {
        return;
    }

    if( h->data )
        free( h->data );
    free( h );
    *ph = NULL;
    move16();
}

/* Initializes a circular buffer (FIFO) with a fixed maximum allowed number of elements */
Word16 JB4_CIRCULARBUFFER_Init( JB4_CIRCULARBUFFER_HANDLE h, Word16 capacity )
{

    /* keep one element free to be able to decide between full/empty buffer */
    capacity = add( capacity, 1 );

    h->data     = malloc( capacity * sizeof( JB4_CIRCULARBUFFER_ELEMENT ) );

    h->capacity = capacity;
    move16();
    h->writePos = 0;
    move16();
    h->readPos  = 0;
    move16();

    return 0;
}

Word16 JB4_CIRCULARBUFFER_Enque( JB4_CIRCULARBUFFER_HANDLE h, JB4_CIRCULARBUFFER_ELEMENT element )
{

    IF( JB4_CIRCULARBUFFER_IsFull( h ) )
    {
        return -1;
    }

    h->data[h->writePos] = element;
    move32();
    h->writePos = add( h->writePos, 1 );

    if( sub( h->capacity, h->writePos ) == 0 )
    {
        h->writePos = 0;
        move16();
    }
    return 0;
}

Word16 JB4_CIRCULARBUFFER_Deque( JB4_CIRCULARBUFFER_HANDLE h, JB4_CIRCULARBUFFER_ELEMENT *pElement )
{

    IF( JB4_CIRCULARBUFFER_IsEmpty( h ) )
    {
        return -1;
    }

    *pElement = h->data[h->readPos];
    move32();
    h->readPos = add( h->readPos, 1 );

    if( sub( h->capacity, h->readPos ) == 0 )
    {
        h->readPos = 0;
        move16();
    }
    return 0;
}

/* Returns the first element. */
JB4_CIRCULARBUFFER_ELEMENT JB4_CIRCULARBUFFER_Front( const JB4_CIRCULARBUFFER_HANDLE h )
{
    JB4_CIRCULARBUFFER_ELEMENT ret;

    ret = L_add(h->data[h->readPos], 0);
    return ret;
}

/* Returns the last element. */
JB4_CIRCULARBUFFER_ELEMENT JB4_CIRCULARBUFFER_Back( const JB4_CIRCULARBUFFER_HANDLE h )
{
    Word16 retPos;

    retPos = sub( h->writePos, 1 );
    if( h->writePos == 0 )
        retPos = sub( h->capacity, 1 );

    return h->data[ retPos ];
}

Word16 JB4_CIRCULARBUFFER_IsEmpty( const JB4_CIRCULARBUFFER_HANDLE h )
{
    Word16 ret;

    ret = 0;
    move16();

    if( sub( h->readPos, h->writePos ) == 0 )
        ret = 1;
    move16();
    return ret;
}

Word16 JB4_CIRCULARBUFFER_IsFull( const JB4_CIRCULARBUFFER_HANDLE h )
{
    Word16 ret;
    Word16 writePosInc;

    writePosInc = add( h->writePos, 1 );
    /* check if writePos++ should wrap around */
    if( sub( writePosInc, h->capacity ) == 0 )
        writePosInc = 0;
    move16();

    ret = 0;
    move16();

    if( sub( writePosInc, h->readPos ) == 0 )
        ret = 1;
    move16();
    return ret;
}

Word16 JB4_CIRCULARBUFFER_Size( const JB4_CIRCULARBUFFER_HANDLE h )
{
    Word16 ret;

    ret = sub( h->writePos, h->readPos );
    /* if wrap around */
    if( ret < 0 )
        ret = add( ret, h->capacity );
    return ret;
}

/* Calculates statistics over all elements: min element */
void JB4_CIRCULARBUFFER_Min( const JB4_CIRCULARBUFFER_HANDLE h, JB4_CIRCULARBUFFER_ELEMENT *pMin )
{
    Word16 upperMinElePos, lowerMinElePos;

    IF( JB4_CIRCULARBUFFER_IsEmpty(h) )
    {
        *pMin = 0;
        move32();
        return;
    }
    BASOP_SATURATE_WARNING_OFF
    IF( sub( h->writePos, h->readPos ) > 0 )
    {
        /* no wraparound */
        /* calc statistics for [readPos;writePos[ */
        upperMinElePos = findIndexOfMinWord32( &(h->data[h->readPos] ), sub( h->writePos, h->readPos ) );
        *pMin = h->data[ add( h->readPos, upperMinElePos ) ];
        move32();
    }
    ELSE
    {
        /* find min for [readPos;capacity[ */
        upperMinElePos = findIndexOfMinWord32( &(h->data[h->readPos] ), sub( h->capacity, h->readPos ) );
        *pMin = h->data[ add( h->readPos, upperMinElePos ) ];
        move32();

        /* special case where writePos is pointing to start of buffer then there are actually no values in
         * lower region so skip lower region find() (find requires at least two elements in the region) */
        IF( h->writePos == 0 )
        {
            BASOP_SATURATE_WARNING_ON
            return;
        }

        /* otherwise find min for [0;writePos[ */
        lowerMinElePos = findIndexOfMinWord32( h->data, h->writePos );

        if( L_sub( *pMin, h->data[lowerMinElePos] ) > 0 )
            *pMin = h->data[ lowerMinElePos ];
        move32();
    }
    BASOP_SATURATE_WARNING_ON
}

/* Calculates statistics over all elements: max element */
void JB4_CIRCULARBUFFER_Max( const JB4_CIRCULARBUFFER_HANDLE h, JB4_CIRCULARBUFFER_ELEMENT *pMax )
{
    Word16 upperMaxElePos, lowerMaxElePos;

    IF( JB4_CIRCULARBUFFER_IsEmpty(h) )
    {
        *pMax = 0;
        move32();
        return;
    }
    BASOP_SATURATE_WARNING_OFF
    IF( sub( h->writePos, h->readPos ) > 0 )
    {
        /* no wraparound */
        /* find max for [readPos;writePos[ */
        upperMaxElePos = findIndexOfMaxWord32( &(h->data[h->readPos] ), sub( h->writePos, h->readPos ) );
        *pMax = h->data[ add( h->readPos, upperMaxElePos ) ];
        move32();
    }
    ELSE
    {
        /* find max for [readPos;capacity[ */
        upperMaxElePos = findIndexOfMaxWord32( &(h->data[h->readPos] ), sub( h->capacity, h->readPos ) );
        *pMax = h->data[ add( h->readPos, upperMaxElePos ) ];
        move32();

        /* special case where writePos is pointing to start of buffer then there are actually no values in
         * lower region so skip lower region find() (find requires at least two elements in the region) */
        IF( h->writePos == 0 )
        {
            BASOP_SATURATE_WARNING_ON
            return;
        }

        /* otherwise find max for [0;writePos[ */
        lowerMaxElePos = findIndexOfMaxWord32( h->data, h->writePos );

        if( L_sub( h->data[lowerMaxElePos], *pMax ) > 0 )
            *pMax = h->data[ lowerMaxElePos ];
        move32();
    }
    BASOP_SATURATE_WARNING_ON
}

/* Calculates statistics over a considered fraction of all elements: min element and percentile */
void JB4_CIRCULARBUFFER_MinAndPercentile( const JB4_CIRCULARBUFFER_HANDLE h, Word32 nElementsToIgnore,
        JB4_CIRCULARBUFFER_ELEMENT *pMin, JB4_CIRCULARBUFFER_ELEMENT *pPercentile )
{
    JB4_CIRCULARBUFFER_ELEMENT minEle;
    JB4_CIRCULARBUFFER_ELEMENT maxElements[100];
    Word32 maxElementsSize;
    Word32 maxElementsCapacity;
    Word32 i;

    /* init output variables */
    minEle = L_add(h->data[h->readPos], 0);

    /* To calculate the percentile, a number of elements with the highest values are collected in maxElements in
     * ascending sorted order. This array has a size of nElementsToIgnore plus one. This additional element is the
     * lowest of all maxElements, and is called the percentile of all elements. */

    maxElementsSize     = L_deposit_l(0);
    maxElementsCapacity = L_add( nElementsToIgnore, 1 );


    BASOP_SATURATE_WARNING_OFF
    IF( L_sub( h->readPos, h->writePos ) <= 0 )
    {
        /* no wrap around */
        /* calc statistics for [readPos;writePos[ */
        FOR( i = h->readPos; i != h->writePos; ++i )
        {
            if( L_sub( h->data[i], minEle ) < 0 )
            {
                minEle = L_add(h->data[i], 0);
            }
            JB4_CIRCULARBUFFER_calcPercentile( maxElements, &maxElementsSize, maxElementsCapacity, h->data[i] );
        }
    }
    ELSE
    {
        /* wrap around */
        /* calc statistics for [readPos;capacity[ */
        FOR( i = h->readPos; i != h->capacity; ++i )
        {
            if( L_sub( h->data[i], minEle ) < 0 )
            {
                minEle = L_add(h->data[i], 0);
            }
            JB4_CIRCULARBUFFER_calcPercentile( maxElements, &maxElementsSize, maxElementsCapacity, h->data[i] );
        }
        /* calc statistics for [0;writePos[ */
        FOR( i = 0; i != h->writePos; ++i )
        {
            if( L_sub( h->data[i], minEle ) < 0 )
            {
                minEle = L_add(h->data[i], 0);
            }
            JB4_CIRCULARBUFFER_calcPercentile( maxElements, &maxElementsSize, maxElementsCapacity, h->data[i] );
        }
    }
    BASOP_SATURATE_WARNING_ON
    *pPercentile = maxElements[0];
    move32();
    *pMin = minEle;
    move32();
}

/* Calculates percentile by selecting greatest elements. */
static void JB4_CIRCULARBUFFER_calcPercentile( JB4_CIRCULARBUFFER_ELEMENT *elements,
        Word32 *size, Word32 capacity, JB4_CIRCULARBUFFER_ELEMENT newElement )
{
    Word32 i,j;

    /* insert newElement if elements buffer is not yet full */
    IF( L_sub( *size, capacity ) < 0 )
    {
        FOR( i = 0; i != *size; ++i )
        {
            IF( L_sub( newElement, elements[i] ) <= 0 )
            {
                /* insert newElement at index i (move all elements above insert pos up a place */
                FOR( j = *size; j >= i; --j  )
                {
                    elements[j+1] = elements[j];
                    move32();
                }

                elements[i] = newElement;
                move32();
                *size = L_add( *size, 1 );
                return;
            }
        }
        /* newElement is maximum, just append it */
        elements[*size] = newElement;
        move32();
        *size = L_add( *size, 1 );
        return;
    }

    /* check if newElement is too small to be inserted in elements buffer */
    IF( L_sub( newElement, elements[0] ) <= 0 )
    {
        return;
    }

    /* select position to insert newElement to elements */
    FOR( i = *size - 1; i != 0; --i )
    {
        IF( L_sub( newElement, elements[i] ) > 0 )
        {
            /* insert newElement at index i (move all elements below insert pos down a place)*/
            FOR( j = 0; j < i; j++  )
            {
                elements[j] = elements[j+1];
                move32();
            }
            elements[i] = newElement;
            move32();
            return;
        }
    }
    /* newElement is just greater than first on in elements buffer */
    elements[0] = newElement;
    move32();
}

