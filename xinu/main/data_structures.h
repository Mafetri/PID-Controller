#ifndef DATA_STRUCTURES
#define DATA_STRUCTURES

#define MAX_SIZE 6

typedef struct queue
{
    unsigned int front;
    unsigned int rear;
    double data[MAX_SIZE][2];
} queue;

typedef struct car_position
{
    double x;
    double y;
    double angle;
    int left_ticks;
    int right_ticks;
    char left_wheel_dir;
    char right_wheel_dir;
} car_position;


void queue_init(queue *q);
int queue_enqueue(queue *q, double value[2]);
int queue_enqueue_values(queue *q, double a, double b);
int queue_dequeue(queue *q, double value[2]);

#endif