#include "queue.h"

void init(Queue *Q)
{
  Q->rear = Q->front = -1;
}

int is_empty(Queue *Q)
{
  return Q->front == Q->rear;
}

int is_full(Queue *Q)
{
  return Q->front == (Q->rear + 1) % SIZE;
}

void enqueue(Queue *Q, char e)
{
  if (is_full(Q))
    printf("Overflow\n");
  else
  {
    Q->rear = (Q->rear + 1) % SIZE;

    Q->data[Q->rear] = e;
  }
}

char dequeue(Queue *Q)
{
  if (is_empty(Q))
  {
    printf("Empty\n");
    return 0;
  }
  else
  {
    Q->front = (Q->front + 1) % SIZE;
    return Q->data[Q->front];
  }
}