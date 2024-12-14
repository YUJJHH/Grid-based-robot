#include <stdio.h>

#define SIZE 300

typedef struct
{
  char data[SIZE];
  int rear, front;
} Queue;

void init(Queue *Q);
int is_empty(Queue *Q);
int is_full(Queue *Q);
void enqueue(Queue *Q, char e);
char dequeue(Queue *Q);
