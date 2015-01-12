/*====================================================================================
    EVS Codec 3GPP TS26.442 Sep 15, 2014. Version 12.0.0
  ====================================================================================*/

/** \file jbm_jb4_inputbuffer.c RTP input buffer with fixed capacity. */

/* system includes */
#include <assert.h>
#include <stdlib.h>
#include <string.h>
/* instrumentation */
/* local includes */
#include "jbm_jb4_inputbuffer.h"
#include "options.h"
#include "stl.h"


/** input buffer with fixed capacity */
struct JB4_INPUTBUFFER
{
    /** elements of input buffer */
    JB4_INPUTBUFFER_ELEMENT *data;
    /** maximum allowed number of elements plus one free element (to decide between full/empty buffer) */
    Word16 capacity;
    /** position of next enque operation */
    Word16 writePos;
    /** position of next deque operation */
    Word16 readPos;
    /** function to compare two elements */
    Word32 (*compareFunction)( const JB4_INPUTBUFFER_ELEMENT first, const JB4_INPUTBUFFER_ELEMENT second,
                               Word16 *replaceWithNewElementIfEqual );
};


/* Creates a input buffer */
Word16 JB4_INPUTBUFFER_Create( JB4_INPUTBUFFER_HANDLE *ph )
{
    JB4_INPUTBUFFER_HANDLE h = malloc( sizeof( struct JB4_INPUTBUFFER ) );

    h->data            = NULL;
    move16();
    h->capacity        = 0;
    move16();
    h->writePos        = 0;
    move16();
    h->readPos         = 0;
    move16();
    h->compareFunction = NULL;
    move16();

    *ph = h;
    move16();
    return 0;
}

/* Destroys the input buffer */
void JB4_INPUTBUFFER_Destroy( JB4_INPUTBUFFER_HANDLE *ph )
{
    JB4_INPUTBUFFER_HANDLE h;

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

/* Initializes a input buffer with a fixed maximum allowed number of elements */
Word16 JB4_INPUTBUFFER_Init( JB4_INPUTBUFFER_HANDLE h, Word16 capacity,
                             Word32 (*compareFunction)( const JB4_INPUTBUFFER_ELEMENT first, const JB4_INPUTBUFFER_ELEMENT second,
                                     Word16 *replaceWithNewElementIfEqual ) )
{

    /* keep one element free to be able to decide between full/empty buffer */
    capacity           = add( capacity, 1 );
    h->data            = malloc( L_mult0( capacity, sizeof( JB4_INPUTBUFFER_ELEMENT ) ) );
    h->capacity        = capacity;
    move16();
    h->writePos        = 0;
    move16();
    h->readPos         = 0;
    move16();
    h->compareFunction = compareFunction;
    move16();
    return 0;
}

Word16 JB4_INPUTBUFFER_Enque( JB4_INPUTBUFFER_HANDLE h, JB4_INPUTBUFFER_ELEMENT element )
{
    Word16 size;
    Word16 low, high, middle;
    Word32 diff;
    Word16 wrapCheck;
    Word16 iterDiff;
    Word16 insertPos;
    Word16 canMoveRight;
    Word16 canMoveLeft;
    Word16 replace;
    Word16 loopIter;

    size = JB4_INPUTBUFFER_Size( h );

    assert( h->capacity > size );

    /* appending the first element is straight forward */
    IF( size == 0 )
    {
        h->data[h->writePos] = element;
        move16();
        h->writePos = add( h->writePos, 1 );

        if( sub( h->writePos, h->capacity ) == 0 )
        {
            h->writePos = 0;
            move16();
        }
        return 0;
    }

    /* there's a high probability that the new element can be appended at the back */
    IF( h->compareFunction( element, JB4_INPUTBUFFER_Back( h ), &replace ) > 0 )
    {
        h->data[h->writePos] = element;
        move16();
        h->writePos = add( h->writePos, 1 );

        if( sub( h->writePos, h->capacity ) == 0 )
        {
            h->writePos = 0;
            move16();
        }
        return 0;
    }

    /* out of order: use binary search to get the position to insert */
    low  = 0;
    move16();
    high = sub( size, 1 );

    WHILE( ( iterDiff = sub( high, low ) ) >= 0 )
    {
        /* equivalent to: middle = low + ( high - low ) / 2; */
        middle = add( low, shr( iterDiff, 1 ) );

        diff = h->compareFunction( element, JB4_INPUTBUFFER_Element( h, middle ), &replace );

        IF( diff < 0 )
        high = sub( middle, 1 );
        ELSE IF( diff > 0 )
        low = add( middle, 1 );
        ELSE   /* an element with same index is already stored */
        {
            IF(replace != 0)
            {
                insertPos = add( h->readPos, middle );
                /* check for wrap around and overwrite pos if required (previously used modulo divide) */
                wrapCheck = sub( insertPos, h->capacity );
                if( wrapCheck >= 0 )
                {
                    insertPos = wrapCheck;
                    move16();
                }
                assert(insertPos == ( h->readPos + middle ) % h->capacity);
                h->data[insertPos] = element;
                move16();
                return 0;
            }
            return 1;
        }
    }

    /* the following checks are for debugging only - excluded from instrumentation */

    insertPos = add( h->readPos, low );
    /* check for wrap around and overwrite pos if required (previously used modulo divide) */
    wrapCheck = sub( insertPos, h->capacity );
    if( wrapCheck >= 0 )
        insertPos = wrapCheck;
    move16();

    canMoveRight = 1;
    move16();
    canMoveLeft  = h->readPos;
    move16();

    IF( sub( h->readPos, h->writePos ) >= 0 )
    {
        canMoveRight = sub( h->writePos, insertPos );
        canMoveLeft  = sub( insertPos, h->writePos );
    }

    assert( canMoveRight > 0 || canMoveLeft > 0 );
    (void)canMoveLeft;

    IF( canMoveRight > 0 )
    {
        /* move higher elements to the right and insert at insertPos */
        FOR( loopIter = 0; loopIter < h->writePos - insertPos; ++loopIter )
        {
            iterDiff = sub( h->writePos, loopIter );
            h->data[ iterDiff ] = h->data[ sub( iterDiff, 1 ) ];
            move16();
        }
        h->data[insertPos] = element;
        move16();
        h->writePos = add( h->writePos, 1 );

        if( sub( h->writePos, h->capacity ) == 0 )
        {
            h->writePos = 0;
            move16();
        }
    }
    ELSE
    {
        /* move lower elements to the left and insert before insertPos */
        FOR( loopIter = 0; loopIter < low; ++loopIter )
        {
            iterDiff = add( h->readPos, loopIter );
            h->data[ sub( iterDiff, 1 ) ] = h->data[iterDiff];
            move16();
        }
        h->data[insertPos-1] = element;
        move16();
        h->readPos = sub( h->readPos, 1 );
        assert( h->readPos >= 0 );
    }
    return 0;
}

Word16 JB4_INPUTBUFFER_Deque( JB4_INPUTBUFFER_HANDLE h, JB4_INPUTBUFFER_ELEMENT *pElement )
{

    IF( JB4_INPUTBUFFER_IsEmpty( h ) )
    {
        return -1;
    }

    *pElement = h->data[h->readPos];
    h->readPos = add( h->readPos, 1 );

    if( sub( h->readPos, h->capacity ) == 0 )
    {
        h->readPos = 0;
        move16();
    }
    return 0;
}

/* Returns the first element. */
JB4_INPUTBUFFER_ELEMENT JB4_INPUTBUFFER_Front( const JB4_INPUTBUFFER_HANDLE h )
{
    JB4_INPUTBUFFER_ELEMENT ret;

    ret = h->data[h->readPos];
    move16();
    return ret;
}

/* Returns the last element. */
JB4_INPUTBUFFER_ELEMENT JB4_INPUTBUFFER_Back( const JB4_INPUTBUFFER_HANDLE h )
{
    JB4_INPUTBUFFER_ELEMENT ret;

    IF( h->writePos != 0 )
    {
        ret = h->data[ sub( h->writePos, 1 ) ];
        move16();
    }
    ELSE
    {
        ret = h->data[ sub( h->capacity, 1 ) ];
        move16();
    }
    return ret;
}

/* Returns the element with the given index (0 means front element). */
JB4_INPUTBUFFER_ELEMENT JB4_INPUTBUFFER_Element( const JB4_INPUTBUFFER_HANDLE h, Word16 index )
{
    JB4_INPUTBUFFER_ELEMENT ret;
    Word16 iter;
    Word16 wrapCheck;

    /* return h->data[(h->readPos + index) % h->capacity] without error handling */
    iter = add( h->readPos, index );
    wrapCheck = sub( iter, h->capacity );
    if( wrapCheck >= 0 )
        iter = wrapCheck;
    move16();
    ret = h->data[ iter ];
    move16();
    return ret;
}

Word16 JB4_INPUTBUFFER_IsEmpty( const JB4_INPUTBUFFER_HANDLE h )
{
    Word16 ret;

    ret = 0;
    move16();
    if( sub( h->readPos, h->writePos ) == 0 )
    {
        ret = 1;
        move16();
    }
    return ret;
}

Word16 JB4_INPUTBUFFER_Size( const JB4_INPUTBUFFER_HANDLE h )
{
    Word16 ret;

    ret = sub( h->writePos, h->readPos );

    /* wrap around */
    if( sub( h->readPos, h->writePos ) > 0 )
        ret = add( ret, h->capacity );
    return ret;
}

