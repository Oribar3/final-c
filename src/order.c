#define _CRT_SECURE_NO_WARNINGS

/*
 * order.c
 * -------
 * Implements order lifecycle management and the FIFO pending-order queue.
 * The queue is an intrusive singly-linked list using the next field
 * embedded in struct Order, avoiding a separate wrapper node allocation.
 */

#include "../include/order.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

Order *create_order(int idx, Position shelf_pos, double arrival_time)
{
    Order *o = (Order *)malloc(sizeof(Order));
    if (!o) {
        fprintf(stderr, "ERROR: Failed to allocate Order.\n");
        exit(EXIT_FAILURE);
    }
    /* Build zero-padded 4-digit order ID, e.g. "O0042" */
    snprintf(o->id, MAX_ID_LEN, "O%04d", idx);
    o->shelf_pos       = shelf_pos;
    o->state           = ORDER_PENDING;
    o->arrival_time    = arrival_time;
    o->completion_time = 0.0;
    o->next            = NULL;
    return o;
}

void destroy_order(Order *order)
{
    if (!order) return;
    free(order);
}

OrderQueue *create_order_queue(void)
{
    OrderQueue *q = (OrderQueue *)malloc(sizeof(OrderQueue));
    if (!q) {
        fprintf(stderr, "ERROR: Failed to allocate OrderQueue.\n");
        exit(EXIT_FAILURE);
    }
    q->head  = NULL;
    q->tail  = NULL;
    q->count = 0;
    return q;
}

void destroy_order_queue(OrderQueue *queue)
{
    if (!queue) return;
    Order *cur = queue->head;
    while (cur) {
        Order *next = cur->next;
        destroy_order(cur);
        cur = next;
    }
    free(queue);
}

void enqueue_order(OrderQueue *queue, Order *order)
{
    order->next = NULL; /* Ensure the new tail has no dangling link */
    if (!queue->head) {
        /* Empty queue: both head and tail point to the single node */
        queue->head = order;
        queue->tail = order;
    } else {
        queue->tail->next = order;
        queue->tail       = order;
    }
    queue->count++;
}

Order *dequeue_order(OrderQueue *queue)
{
    if (!queue->head) return NULL;

    Order *o    = queue->head;
    queue->head = o->next;
    if (!queue->head) {
        /* Queue is now empty; reset tail to avoid a dangling pointer */
        queue->tail = NULL;
    }
    o->next = NULL; /* Caller receives a clean, unlinked node */
    queue->count--;
    return o;
}

int is_queue_empty(const OrderQueue *queue)
{
    return queue->count == 0;
}
