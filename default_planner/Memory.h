#ifndef memory_h
#define memory_h

#include <vector>
#include <iostream>

#include "search_node.h"

namespace TrafficMAPF{

class MemoryPool
{
    //every location in the map has a node with id equal to the location
public:
    MemoryPool(){
        size = 0;
        index = 0;
        nodes = nullptr;
        label = 0;
    };
    MemoryPool(int size){
        init(size);
    };

    int generated(){
        return index;
    }

    void init(int size){
        this->size = size;
        index = 0;
        label = 0;
        nodes = new s_node[size];
        ready = true;
    }

    bool is_ready(){
        return ready;
    }

    bool has_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        return nodes[id].label == label && nodes[id].id == id;
    }
    bool is_closed(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label){
            return false;
        }
        return nodes[id].is_closed();
    }
    s_node* get_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1){
            std::cout << "error node not generated yet" << std::endl;
            exit(1);
        }
        return &(nodes[id]);
    }
    void close_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].label != label || nodes[id].id == -1){
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        nodes[id].close();
    }
    s_node* generate_node(int id, int g, int h, int op_flow, int depth, int all_vertex_flow = 0){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        
        if (nodes[id].label == label && nodes[id].id != -1){
            std::cout << "node already generated " << id << ","<< is_ready()<< std::endl;

            std::cout << "node already generated " << nodes[id].id<< std::endl;
            exit(1);
        }
        nodes[id].reset();
        nodes[id].label = label;
        nodes[id].id = id;
        nodes[id].g = g;
        nodes[id].h = h;
        nodes[id].op_flow = op_flow;
        nodes[id].depth = depth;
        nodes[id].all_vertex_flow = all_vertex_flow;
        index++;
        return &(nodes[id]);
    }

    void free_node(int id){
        if (id >= size)
        {
            std::cout << "range out of memory pool size "<< id<<","<<index<<","<<size << std::endl;
            exit(1);
        }
        if (nodes[id].id == -1){
            std::cout << "node not generated yet" << std::endl;
            exit(1);
        }
        nodes[id].reset();
        index--;
    }

    void reset(){
        index = 0;
        label++;

    }

    ~MemoryPool(){
        if (nodes != nullptr){
            delete [] nodes;
            nodes = nullptr;
        }
    }
private:
    s_node* nodes;
    int size;
    int index;
    int label;
    bool ready = false;
};


}
#endif