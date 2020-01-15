#ifndef __LIST_H__
#define __LIST_H__
#include <assert.h>

typedef struct list_head_s {
  struct list_head_s *prev;
  struct list_head_s *next;
} list_h_t;

#define LIST_HEAD_INIT(name) { &(name), &(name) }
#define LIST_HEAD(name) \
    list_h_t name = LIST_HEAD_INIT(name)

#define DEBUG_LIST
#ifdef DEBUG_LIST
#define ASSERT_LIST(p) assert(((p)!=NULL) && (sizeof(*(p))==sizeof(list_h_t)))
#else
#define ASSERT_LIST(p)
#endif

#if 0
static void INIT_LIST_HEAD(list_h_t *list)
{
    list->next = list;
    list->prev = list;
}
#else
#define INIT_LIST_HEAD(head) { \
    ASSERT_LIST(head); \
    (head)->next = (head); \
    (head)->prev = (head); }
#endif

#if 0
static void INIT_LIST_HEAD(list_h_t *list)
static void list_add_tail(list_h_t *entry, list_h_t *head)
{
    entry->next = head;
    entry->prev = head->prev;
    head->prev->next = entry;
    head->prev = entry;
}
#else
#define list_add_tail(entry, head) { \
    ASSERT_LIST(entry); \
    ASSERT_LIST(head); \
    (entry)->next = (head); \
    (entry)->prev = (head)->prev; \
    (head)->prev->next = (entry); \
    (head)->prev = (entry); }
#endif

#if 0
static void list_del(list_h_t *entry)
{
    entry->next->prev = entry->prev;
    entry->prev->next = entry->next;
    entry->next = NULL;
    entry->prev = NULL;
}
#else
#define list_del(entry) { \
    ASSERT_LIST(entry); \
    (entry)->next->prev = (entry)->prev; \
    (entry)->prev->next = (entry)->next; \
    (entry)->next = NULL; \
    (entry)->prev = NULL; }
#endif

#if 0
static int list_is_last(list_h_t *list, list_h_t *head)
{
    return ((list->next == head) || (list->prev == head));
}
#else
#define list_is_last(list, head) (((list)->next == (head)) ||\
                                  ((list)->prev == (head)))
#endif
#if 0
static inline int list_empty(list_h_t *head)
{
    return ((head->next == head) || (head->prev == head));
}
#else
#define list_empty(head) (((head)->next == (head)) || ((head)->prev == (head)))
#endif


/**
 * list_for_each_safe - iterate over a list safe against removal of list entry
 * @pos:    the &struct list_head to use as a loop cursor.
 * @n:      another &struct list_head to use as temporary storage
 * @head:   the head for your list.
 */
#define list_for_each_safe(pos, n, head) \
    for (pos = (head)->next, n = pos->next; pos != (head); \
         pos = n, n = pos->next)

/* get the tail node */
#define list_tail(head) ((head)->prev)

/* get the head node */
#define list_head(head) ((head)->next)

#endif /* __LIST_H__ */
