#ifndef PRIORITYQUEUE_H_
#define PRIORITYQUEUE_H_

typedef struct item {
	float priority;
	int value;
} item;

typedef struct queue {
	int size;
	unsigned int allocated;
	item *root;
	int *index;
	unsigned int indexAllocated;
} queue;

void insert (queue *q, int value, float priority);
void deleteMin (queue *q);
item *findMin (const queue *q);
void changePriority (queue *q, int ind, float newPriority);
void delete (queue *q, int ind);
int priorityOf (const queue *q, int ind);
int exists (const queue *q, int ind);
queue *createQueue ();
void freeQueue (queue *q);

#endif
