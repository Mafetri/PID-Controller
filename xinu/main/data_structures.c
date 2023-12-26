#include "data_structures.h"

void queue_init(queue *q)
{
    q->front = 0;
    q->rear = 0;
}

int queue_enqueue(queue *q, double value[2])
{
    if (q->rear == MAX_SIZE)
    {
        return -1;
    }

    for (int i = 0; i < 2; i++) {
        q->data[q->rear][i] = value[i];
    }
    q->rear = (q->rear + 1) % MAX_SIZE;

    return 1;
}

int queue_enqueue_values(queue *q, double a, double b)
{
    if (q->rear == MAX_SIZE)
    {
        return -1;
    }

    q->data[q->rear][0] = a;
    q->data[q->rear][1] = b;
    q->rear = (q->rear + 1) % MAX_SIZE;

    return 1;
}

int queue_dequeue(queue *q, double value[2])
{
    if (q->front == q->rear)
    {
        return -1;
    }

    for (int i = 0; i < 2; i++) {
        value[i] = q->data[q->front][i];
    }
    q->front = (q->front + 1) % MAX_SIZE;

    return 1;
}