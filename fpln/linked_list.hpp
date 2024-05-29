#pragma once

#include <cstdint>
#include <stack>


namespace struct_util
{
    template <class T>
    struct list_node_t
    {
        list_node_t *prev, *next;
        T data;
    };

    template <class T>
    struct linked_list_t
    {
        list_node_t<T> head, tail;
        std::size_t size;


        linked_list_t();

        void push_front(list_node_t<T> *node);

        void push_back(list_node_t<T> *node);

        /*
            Function:
            insert_before
            @param *node: node before which to insert
            @param *node_insert: node to be inserted
        */

        void insert_before(list_node_t<T> *node, list_node_t<T> *node_insert);

        void pop(list_node_t<T> *node, std::stack<list_node_t<T>*>& release_stack);

        void release_all(std::stack<list_node_t<T>*>& release_stack);
    };

    template <class T>
    struct ll_node_stack_t
    {
        T *nodes;
        std::stack<T*> ptr_stack;


        ll_node_stack_t(std::size_t sz);

        void destroy();
    };


    // linked_list_t definitions:

    template <class T>
    linked_list_t<T>::linked_list_t()
    {
        head.next = &tail;
        head.prev = nullptr;

        tail.prev = &head;
        tail.next = nullptr;

        size = 0;
    }

    template <class T>
    void linked_list_t<T>::push_front(list_node_t<T> *node)
    {
        node->next = head.next;
        node->prev = &head;
        head.next = node;
        size++;
    }

    template <class T>
    void linked_list_t<T>::push_back(list_node_t<T> *node)
    {
        node->prev = tail.prev;
        node->next = &tail;
        tail.prev = node;
        size++;
    }

    template <class T>
    void linked_list_t<T>::insert_before(list_node_t<T> *node, list_node_t<T> *node_insert)
    {
        node_insert->prev = node->prev;
        node_insert->next = node;
        node->prev = node_insert;
        size++;
    }

    template <class T>
    void linked_list_t<T>::pop(list_node_t<T> *node, 
        std::stack<list_node_t<T>*>& release_stack)
    {
        if(node->prev != nullptr && node->next != nullptr)
        {
            node->prev->next = node->next;
            node->next->prev = node->prev;
            release_stack.push(node);
            size--;
        }
    }

    template <class T>
    void linked_list_t<T>::release_all(std::stack<list_node_t<T>*>& release_stack)
    {
        list_node_t<T>* curr = head.next;

        while(curr != &tail)
        {
            release_stack.push(curr);
            curr = curr->next;
        }

        size = 0;
    }

    // ll_node_stack_t definitions:

    template <class T>
    ll_node_stack_t<T>::ll_node_stack_t(std::size_t sz)
    {
        nodes = new T[sz];
        for(std::size_t i = 0; i < sz; i++)
        {
            ptr_stack.push(nodes + i);
        }
    };

    template <class T>
    void ll_node_stack_t<T>::destroy()
    {
        ptr_stack.empty();
        delete[] nodes;
    };
} // namespace struct_util
