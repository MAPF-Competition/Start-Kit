#ifndef WARTHOG_PQUEUE_H
#define WARTHOG_PQUEUE_H

// pqueue.h
//
// A min priority queue. Loosely based on an implementation from HOG
// by Nathan Sturtevant.
//
// @author: dharabor
// @created: 09/08/2012
//
// Modified for search nodes used here. Mike

#include "utils.h"
#include "Memory.h"
#include "search_node.h"
#include "Types.h"
#include <cassert>
#include <iostream>

namespace DefaultPlanner{


struct min_q 
{ static const bool is_min_ = true; };

struct max_q
{ static const bool is_min_ = false; };


template <
        class s_node = s_node,
        class Comparator = cmp_less_f,
        class QType = min_q>
class pqueue 
{
	public:
        pqueue(Comparator* cmp, unsigned int size=1024)
            : pqueue(size)
        { cmp_ = cmp; }

		pqueue(unsigned int size=1024)
            : maxsize_(size), minqueue_(QType().is_min_), queuesize_(0), 
            elts_(0), heap_ops_(0)
        { resize(size); }

        ~pqueue()
        { delete [] elts_; }
        

		// removes all elements from the pqueue
        void
        clear()
        { 
            queuesize_ = 0; 
            heap_ops_ = 0;
        }

		// reprioritise the specified element (up or down)
        void 
        decrease_key(s_node* val)
        {	
            assert(val->get_priority() < queuesize_);
            minqueue_ ?  
                heapify_up(val->get_priority()) :
                heapify_down(val->get_priority());
        }

        void 
        increase_key(s_node* val)
        {
            assert(val->get_priority() < queuesize_);
            minqueue_ ? 
                heapify_down(val->get_priority()) :
                heapify_up(val->get_priority());
        }

		// add a new element to the pqueue
        void 
        push(s_node* val)
        {
            if(contains(val))
            {
                return;
            }

            if(queuesize_+1 > maxsize_)
            {
                resize(maxsize_*2);
            }
            unsigned int priority = queuesize_;
            elts_[priority] = val;
            val->set_priority(priority);
            queuesize_++;
            heapify_up(priority);
        }

		// remove the top element from the pqueue
        s_node*
        pop()
        {
            if (queuesize_ == 0)
            {
                return 0;
            }

            s_node *ans = elts_[0];
            queuesize_--;

            if(queuesize_ > 0)
            {
                elts_[0] = elts_[queuesize_];
                elts_[0]->set_priority(0);
                heapify_down(0);
            }
            return ans;
        }

        s_node* top(){
            if (queuesize_ == 0)
            {
                return 0;
            }
            return elts_[0];
        }

		// @return true if heap contains search node @param n
		// and return false if it does not 
		inline bool
		contains(s_node* n)
		{
			unsigned int index = n->get_priority();
			if(index < queuesize_ && &*n == &*elts_[index])
			{
				return true;
			}
			return false;
		}

		// retrieve the top element without removing it
		inline s_node*
		peek()
		{
			if(queuesize_ > 0)
			{
				return this->elts_[0];
			}
			return 0;
		}

        uint32_t
        get_heap_ops()
        { return heap_ops_; }

		inline unsigned int
		size()
		{ return queuesize_; }

        inline unsigned int
		empty()
		{ return queuesize_ ==0; }

		inline bool
		is_minqueue() 
		{ return minqueue_; } 
		
        void 
        print(std::ostream& out)
        {
            for(unsigned int i=0; i < queuesize_; i++)
            {
                elts_[i]->print(out);
                out << std::endl;
            }
        }

		size_t
		mem()
		{
			return maxsize_*sizeof(s_node*)
				+ sizeof(*this);
		}

	private:
		unsigned int maxsize_;
		bool minqueue_;
		unsigned int queuesize_;
		s_node** elts_;
        Comparator* cmp_;
        uint32_t heap_ops_;

		// reorders the subpqueue containing elts_[index]
        void 
        heapify_up(unsigned int index)
        {
            heap_ops_++;
            assert(index < queuesize_);
            while(index > 0)
            {
                unsigned int parent = (index-1) >> 1;
                if((*cmp_)(*elts_[index], *elts_[parent]))
                //if(*elts_[index] < *elts_[parent])
                {
                    swap(parent, index);
                    index = parent;
                }
                else { break; }
            }
        }
		
		// reorders the subpqueue under elts_[index]
        void 
        heapify_down(unsigned int index)
        {
            heap_ops_++;
            unsigned int first_leaf_index = queuesize_ >> 1;
            while(index < first_leaf_index)
            {
                // find smallest (or largest, depending on heap type) child
                unsigned int child1 = (index<<1)+1;
                unsigned int child2 = (index<<1)+2;
                unsigned int which = child1;
                if((child2 < queuesize_) && 
                    (*cmp_)(*elts_[child2], *elts_[child1])) 
                   //*elts_[child2] < *elts_[child1])
                { which = child2; }

                // swap child with parent if necessary
                if((*cmp_)(*elts_[which], *elts_[index]))
                //if(*elts_[which] < *elts_[index])
                {
                    swap(index, which);
                    index = which;
                }
                else { break; }
            }
        }

		// allocates more memory so the pqueue can grow
        void
        resize(unsigned int newsize)
        {
            if(newsize < queuesize_)
            {
                std::cerr << "err; pqueue::resize newsize < queuesize " << std::endl;
                exit(1);
            }

            s_node** tmp = new s_node*[newsize];
            for(unsigned int i=0; i < queuesize_; i++)
            {
                tmp[i] = elts_[i];
            }
            delete [] elts_;
            elts_ = tmp;
            maxsize_ = newsize;
        }

		// swap the positions of two nodes in the underlying array
		inline void 
		swap(unsigned int index1, unsigned int index2)
		{
			assert(index1 < queuesize_ && index2 < queuesize_);

			s_node* tmp = elts_[index1];
			elts_[index1] = elts_[index2];
			elts_[index1]->set_priority(index1);
			elts_[index2] = tmp;
			tmp->set_priority(index2);
		}
};

typedef pqueue<s_node,cmp_less_of, min_q> pqueue_min_of;

typedef pqueue<s_node,cmp_less_f, min_q> pqueue_min_f;


}

#endif