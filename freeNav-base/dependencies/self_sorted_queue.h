//
// Created by yaozhuo on 2022/5/14.
//

#ifndef FREENAV_BASE_SELF_SORTED_QUEUE_H
#define FREENAV_BASE_SELF_SORTED_QUEUE_H

#include "../basic_elements/point.h"
#include <vector>
#include <assert.h>
#include <memory>
/*
 * the queue use in DFS for auto sorted
 * */
namespace freeNav {

    template <typename VALUE>
    struct withIndex {
        withIndex(const VALUE& val, const std::shared_ptr<NodeId>& index_in_queue)
                           :val_(val),index_in_queue_(index_in_queue) {
            if(index_in_queue == nullptr)
                index_in_queue_ = std::make_shared<NodeId>(MAX<NodeId>);
        }
        std::shared_ptr<NodeId> index_in_queue_ = nullptr;
        VALUE val_;
    };

    /* T1 must be a comparable type  */
    template<typename KEY, typename VALUE>
    using SortPair = std::pair<KEY, withIndex<VALUE> >;

    /* O(n^2) sort */
    template<typename KEY, typename VALUE>
    class AutoSortQueue {
    public:
        AutoSortQueue() {}

        /* insert a minor value */
        // return the location in queue
        int insertMin(const SortPair<KEY, VALUE>& element) {
            if(queue_.empty()) {
                queue_.push_back(element);
                return 0;
            }
            /* TODO: replace with logN insert */
            for(auto iter = queue_.end() - 1; iter != queue_.begin(); iter--) {
                if((*iter).first > element.first) {
                    iter ++;
                    queue_.insert(iter, element);
                    return iter - queue_.begin();
                }
            }
            if(element.first > queue_[0].first) {
                queue_.insert(queue_.begin(), element);
                return 0;
            } else {
                queue_.insert(queue_.begin() + 1, element);
                return 1;
            }
        }

        SortPair<KEY, VALUE> popMin() {
            emptyQueueCheck();
            auto retv = queue_.back();
            queue_.pop_back();
            return retv;
        }

        const SortPair<KEY, VALUE>& getMin() const {
            emptyQueueCheck();
            return queue_.back();
        }

        const std::vector<SortPair<KEY, VALUE> > & queue() const {
            return queue_;
        }

        static std::string toValueString(const std::vector<SortPair<KEY, VALUE> > & queue) {
            if(queue.empty()) return "";
            std::stringstream ssr;
            for(const auto& pair : queue) {
                ssr << "(" << pair.first << "," << pair.second.val_ << " | id: " << * (pair.second.index_in_queue_) << ") > ";
            }
            return ssr.str();
        }

    private:

        void emptyQueueCheck() {
            if(queue_.empty()) {
                std::cout << "ASQ::pM: empty queue" << std::endl;
                exit(0);
            }
        }

        std::vector<SortPair<KEY, VALUE> > queue_;

    };


//    template<typename KEY, typename VALUE>
//    using SortablePair = std::pair<KEY, VALUE>;

    template<typename KEY, typename VALUE>
    using SortQueue = std::vector<SortPair<KEY, VALUE> >;


    int HeapInsert(int *heap, int n, int num);
    int HeapDelete(int *heap, int n);
    int HeapAdjust(int *heap, int top, int n);
    int HeapSort(int *heap, int n);
    int CreatHeap(int *array, int n);



    /*
     * 堆插入算法。(小顶堆)
     * 先将num插入堆尾，易知从新数据的父结点到根结点是一个有序的序列，
     * 将num插入到该有序序列当中，该过程为直接插入排序。
     * 未插入前数据长度为n。
     */
    template<typename KEY, typename VALUE>
    int HeapInsert(SortQueue<KEY, VALUE>& heap, int n, const SortPair<KEY, VALUE>& num)
    {
        int i, j;

        heap[n] = num;//num插入堆尾
        i = n;
        j = (n - 1) / 2;//j指向i的父结点

        //注意不要漏掉i!=0的条件。因为必须保证i有父结点j。j>=0并不能保证i!=0。
        //如果没有此条件，当i=0时，j=0,若heap[0]>num,程序就会陷入死循环。
        while (j >= 0 && i != 0)
        {
            if (heap[j].first <= num.first)
                break;
            heap[i] = heap[j];
            i = j;
            j = (i - 1) / 2;
        }
        heap[i] = num;

        return 0;
    }


    /*
     * 堆调整算法。(小顶堆)
     * 已知heap[top]结点的左右子树均为堆，调整堆中元素，使以heap[top]为根结点的树为堆。
     * n为堆中元素总数。
     */
    template<typename KEY, typename VALUE>
    int HeapAdjust(SortQueue<KEY, VALUE>& heap, int top, int n)
    {
        int j = 2 * top + 1;    //左孩子结点
        SortPair<KEY, VALUE> temp = heap[top];

        while (j < n)
        {
            if (j + 1 < n && heap[j + 1].first < heap[j].first)
                j++;    //使j指向左右孩子中较小的结点。
            if (heap[j].first >= temp.first)
                break;
            heap[top] = heap[j];
            top = j;
            j = 2 * top + 1;
        }
        heap[top] = temp;
        return 0;
    }

    /*
     * 堆删除算法。(删除堆顶元素)
     * n表示未删除前堆中数据的总数。
     */
    template<typename KEY, typename VALUE>
    int HeapDelete(SortQueue<KEY, VALUE>& heap, int n)
    {
        //使用堆尾元素直接覆盖堆顶元素。
        heap[0] = heap[n - 1];
        //从堆顶到堆尾(此时堆中只有n-1个元素)进行堆调整。
        HeapAdjust(heap, 0, n - 1);
        return 0;
    }


    /*
     * 堆排序算法。
     * 形参heap为大顶堆时，实现的是由小到大；
     * 形参heap为小顶堆时，实现的是由大到小；
     */
    template<typename KEY, typename VALUE>
    int HeapSort(SortQueue<KEY, VALUE>& heap, int n)
    {
        int i;
        SortPair<KEY, VALUE> temp;

        for (i = n - 1; i > 0; i--)
        {
            //将堆顶元素和未排序的最后一个元素交换。
            temp = heap[0];
            heap[0] = heap[i];
            heap[i] = temp;
            //交换之后进行堆调整
            HeapAdjust(heap, 0, i);
        }
        return 0;
    }

    /*
     * 建堆算法。
     * 将无序数组array[]转换为堆。
     */
    template<typename KEY, typename VALUE>
    int CreatHeap(SortQueue<KEY, VALUE>& heap, int n)
    {
        int i;
        //最后一个结点的编号为n-1，该结点的父节点(n-2)/2为最后一个非终端结点。
        //从结点(n-2)/2到根结点，依次进行堆调整。
        for (i = (n - 2) / 2; i >= 0; i--)
        {
            HeapAdjust(heap, i, n);
        }
        return 0;
    }

    // key for sort, value for store
    template<typename KEY, typename VALUE>
    class AutoSortHeap {
    public:

        explicit AutoSortHeap(int max_size) {
            heap_ = SortQueue<KEY, VALUE>(max_size, {MAX<KEY>, withIndex<VALUE>(VALUE(), nullptr)});
        }

        void reset() {
            for(auto& element : heap_) {
                if(*element.second.index_in_queue_ != MAX<NodeId>) {
                    element = {MAX<KEY>, withIndex<VALUE>(VALUE(), nullptr)};
                } else {
                    break;
                }
            }


            // check, not all reseted!
//            for(auto& element : heap_) {
//                if(element.second != nullptr || element.first != MAX<KEY>) {
//                    std::cout << element.first << "/ " << element.second << " not reset" << std::endl;
//                }
//            }
            //heap_ = SortQueue<KEY, VALUE>(heap_.size(), {MAX<KEY>, VALUE()});

            count_ = 0;
//            std::pair<KEY, VALUE> default_val = {MAX<KEY>, VALUE()};
//            std::fill(heap_.begin(), heap_.end(), default_val);// ?
        }

        int HeapInsert(const SortPair<KEY, VALUE>& element)
        {

//            if(element.first == MAX<KEY> || element.second == nullptr) {
//                std::cout << element.first << "/ " << element.second << " not reset" << std::endl;
//            }
            int i, j;
            if(count_ == heap_.size()) {
                std::cout << __FUNCTION__ << " heap over size " << heap_.size() << std::endl;
            }
            heap_[count_] = element;//num插入堆尾
            * heap_[count_].second.index_in_queue_ = count_;
            i = count_;
            j = (count_ - 1) / 2;//j指向i的父结点

            //注意不要漏掉i!=0的条件。因为必须保证i有父结点j。j>=0并不能保证i!=0。
            //如果没有此条件，当i=0时，j=0,若heap[0]>element,程序就会陷入死循环。
            while (j >= 0 && i != 0)
            {
                if (heap_[j].first <= element.first)
                    break;
                heap_[i] = heap_[j];
                * heap_[i].second.index_in_queue_ = i;
                i = j;
                j = (i - 1) / 2;
            }
            heap_[i] = element;
            * heap_[i].second.index_in_queue_ = i;
            count_ ++;
            return 0;
        }

        SortPair<KEY, VALUE> HeapPopMin()
        {
            if(count_ <= 0) return {MAX<KEY>, withIndex<VALUE>(VALUE(), nullptr)};
            //使用堆尾元素直接覆盖堆顶元素。
            auto retv = heap_[0];
            heap_[0] = heap_[count_ - 1];
            * heap_[0].second.index_in_queue_ = 0;
            //从堆顶到堆尾(此时堆中只有n-1个元素)进行堆调整。
            count_ --;
            HeapAdjust(0);
            * retv.second.index_in_queue_ = MAX<NodeId>;
            return retv;
        }

        const SortPair<KEY, VALUE>& HeapGetMin() const
        {
            if(count_ == 0) {
                return {MAX<KEY>, VALUE()};
            }
            //使用堆尾元素直接覆盖堆顶元素。
            auto retv = heap_[0];
            return retv;
        }

        std::string toKeysString() {
            if(heap_.empty()) return "";
            std::stringstream ssr;
            for(int i=0; i<count_; i++) {
                const auto& pair = heap_[i];
                ssr << "(" << pair.first << "," << pair.second.val_ << " | id: " << * (pair.second.index_in_queue_)
                << ") > ";
            }
            return ssr.str();
        }

        bool isEmpty() const {
            return count_ <= 0;
        }

        int size() const {
            return count_;
        }

        void updateValue(int index, const KEY& key) {
            if(index >= count_) {
                std::cout << "index " << index << " over size " << count_ << std::endl;
                return;
            }
            heap_[index].first = key;
            HeapAdjustWhenBecomeSmaller(index);
        }

        void HeapAdjustWhenBecomeSmaller(int index) {
            KEY key = heap_[index].first;
            int temp_index = index;
            while(1) {
                int parent_index = temp_index == 1 ? 0 : temp_index/2;
                if(heap_[parent_index].first > key) {
                    std::swap(heap_[parent_index], heap_[temp_index]);
                    *heap_[temp_index].second.index_in_queue_   = temp_index;
                    *heap_[parent_index].second.index_in_queue_ = parent_index;
                }
                if(parent_index == 0) break;
                temp_index = parent_index;
            }
        }


    private:

        int HeapAdjust(int top)
        {
            int j = 2 * top + 1;    //左孩子结点
            SortPair<KEY, VALUE> temp = heap_[top];

            while (j < count_)
            {
                if (j + 1 < count_ && heap_[j + 1].first < heap_[j].first)
                    j++;    //使j指向左右孩子中较小的结点。
                if (heap_[j].first >= temp.first)
                    break;
                heap_[top] = heap_[j];
                *heap_[top].second.index_in_queue_ = top;
                top = j;
                j = 2 * top + 1;
            }
            heap_[top] = temp;
            *heap_[top].second.index_in_queue_ = top;
            return 0;
        }

        SortQueue<KEY, VALUE> heap_;
        int count_ = 0;
    };

    template<typename KEY, typename VALUE>
    class AutoSortHeap;

    template<typename KEY, typename VALUE>
    using AutoSortHeapPtr = std::shared_ptr<AutoSortHeap<KEY, VALUE> >;

}
#endif //FREENAV_SELF_SORTED_QUEUE_H
