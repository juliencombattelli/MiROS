#include "circular_buffer.h"
#include "qassert.h"

Q_DEFINE_THIS_FILE

static inline size_t advance_headtail_value(size_t value, size_t max)
{
	return (value + 1) % max;
}

static void advance_head_pointer(circular_buf_t* me)
{
	Q_ASSERT(me);

	if(circular_buf_full(me))
	{
		me->tail = advance_headtail_value(me->tail, me->max);
	}

	me->head = advance_headtail_value(me->head, me->max);
	me->full = (me->head == me->tail);
}

void circular_buf_init(circular_buf_t* buf, uint8_t* buffer, size_t size)
{
	Q_ASSERT(buffer && size);
	Q_ASSERT(buf);

	buf->buffer = buffer;
	buf->max = size;
	circular_buf_reset(buf);

	Q_ASSERT(circular_buf_empty(buf));
}

void circular_buf_free(circular_buf_t* me)
{
}

void circular_buf_reset(circular_buf_t* me)
{
	Q_ASSERT(me);

	me->head = 0;
	me->tail = 0;
	me->full = false;
}

size_t circular_buf_size(circular_buf_t* me)
{
	Q_ASSERT(me);

	size_t size = me->max;

	if(!circular_buf_full(me))
	{
		if(me->head >= me->tail)
		{
			size = (me->head - me->tail);
		}
		else
		{
			size = (me->max + me->head - me->tail);
		}
	}

	return size;
}

size_t circular_buf_capacity(circular_buf_t* me)
{
	Q_ASSERT(me);

	return me->max;
}

void circular_buf_put(circular_buf_t* me, uint8_t data)
{
	Q_ASSERT(me && me->buffer);

	me->buffer[me->head] = data;

	advance_head_pointer(me);
}

int circular_buf_try_put(circular_buf_t* me, uint8_t data)
{
	int r = -1;

	Q_ASSERT(me && me->buffer);

	if(!circular_buf_full(me))
	{
		me->buffer[me->head] = data;
		advance_head_pointer(me);
		r = 0;
	}

	return r;
}

int circular_buf_get(circular_buf_t* me, uint8_t* data)
{
	Q_ASSERT(me && data && me->buffer);

	int r = -1;

	if(!circular_buf_empty(me))
	{
		*data = me->buffer[me->tail];
		me->tail = advance_headtail_value(me->tail, me->max);
		me->full = false;
		r = 0;
	}

	return r;
}

bool circular_buf_empty(circular_buf_t* me)
{
	Q_ASSERT(me);

	return (!circular_buf_full(me) && (me->head == me->tail));
}

bool circular_buf_full(circular_buf_t* me)
{
	Q_ASSERT(me);

	return me->full;
}

int circular_buf_peek(circular_buf_t* me, uint8_t* data, unsigned int look_ahead_counter)
{
	int r = -1;
	size_t pos;

	Q_ASSERT(me && data && me->buffer);

	// We can't look beyond the current buffer size
	if(circular_buf_empty(me) || look_ahead_counter > circular_buf_size(me))
	{
		return r;
	}

	pos = me->tail;
	for(unsigned int i = 0; i < look_ahead_counter; i++)
	{
		data[i] = me->buffer[pos];
		pos = advance_headtail_value(pos, me->max);
	}

	return 0;
}
