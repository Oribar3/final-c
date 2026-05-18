#ifndef ORDER_H
#define ORDER_H

/*
 * order.h
 * -------
 * Declares the API for Order creation and for the intrusive FIFO linked-list
 * queue that holds orders awaiting an available robot.
 *
 * The Order struct (defined in types.h) embeds a next pointer so it can act
 * as its own queue node without a separate wrapper allocation.
 *
 * Depends on: types.h
 */

#include "types.h"

/*
 * create_order
 * ------------
 * Heap-allocates and initialises an Order.
 *
 * Parameters:
 *   idx          - numeric index used to build the ID string "O%04d"
 *   shelf_pos    - grid position of the target shelf
 *   arrival_time - simulation clock value when the order was created
 *
 * Returns:
 *   Order* with state=ORDER_PENDING, next=NULL. Exits on allocation failure.
 */
Order *create_order(int idx, Position shelf_pos, double arrival_time);

/*
 * destroy_order
 * -------------
 * Frees a single Order node. Does NOT traverse the next pointer.
 * Callers are responsible for unlinking the node from any list first.
 *
 * Parameters:
 *   order - Order to free; no-op if NULL.
 */
void destroy_order(Order *order);

/*
 * create_order_queue
 * ------------------
 * Heap-allocates an empty OrderQueue (head=NULL, tail=NULL, count=0).
 *
 * Returns: OrderQueue*; exits on allocation failure.
 */
OrderQueue *create_order_queue(void);

/*
 * destroy_order_queue
 * -------------------
 * Walks the queue calling destroy_order on each node, then frees the struct.
 *
 * Parameters:
 *   queue - may be NULL (no-op).
 */
void destroy_order_queue(OrderQueue *queue);

/*
 * enqueue_order
 * -------------
 * Appends order to the tail of queue (FIFO).
 * order->next is set to NULL before linking so callers need not pre-clear it.
 *
 * Parameters:
 *   queue - valid, non-NULL OrderQueue*
 *   order - valid Order*
 */
void enqueue_order(OrderQueue *queue, Order *order);

/*
 * dequeue_order
 * -------------
 * Removes and returns the head Order (FIFO).
 * Sets the returned node's next pointer to NULL so the caller has a clean node.
 *
 * Parameters:
 *   queue - valid OrderQueue*
 *
 * Returns: Order* (caller owns it); NULL if queue is empty.
 */
Order *dequeue_order(OrderQueue *queue);

/*
 * is_queue_empty
 * --------------
 * Returns 1 if queue->count == 0, 0 otherwise.
 *
 * Parameters:
 *   queue - valid OrderQueue*
 */
int is_queue_empty(const OrderQueue *queue);

#endif /* ORDER_H */
