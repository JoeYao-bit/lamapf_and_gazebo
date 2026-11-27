//
// Created by yaozhuo on 2022/5/14.
//

#include "self_sorted_queue.h"

namespace freeNav {


    /*
     * 堆插入算法。(小顶堆)
     * 先将num插入堆尾，易知从新数据的父结点到根结点是一个有序的序列，
     * 将num插入到该有序序列当中，该过程为直接插入排序。
     * 未插入前数据长度为n。
     */
    int HeapInsert(int *heap, int n, int num)
    {
        int i, j;

        heap[n] = num;//num插入堆尾
        i = n;
        j = (n - 1) / 2;//j指向i的父结点

        //注意不要漏掉i!=0的条件。因为必须保证i有父结点j。j>=0并不能保证i!=0。
        //如果没有此条件，当i=0时，j=0,若heap[0]>num,程序就会陷入死循环。
        while (j >= 0 && i != 0)
        {
            if (heap[j] <= num)
                break;
            heap[i] = heap[j];
            i = j;
            j = (i - 1) / 2;
        }
        heap[i] = num;

        return 0;
    }

    /*
     * 堆删除算法。(删除堆顶元素)
     * n表示未删除前堆中数据的总数。
     */
    int HeapDelete(int *heap, int n)
    {
        //使用堆尾元素直接覆盖堆顶元素。
        heap[0] = heap[n - 1];
        //从堆顶到堆尾(此时堆中只有n-1个元素)进行堆调整。
        HeapAdjust(heap, 0, n - 1);
        return 0;
    }

    /*
     * 堆调整算法。(小顶堆)
     * 已知heap[top]结点的左右子树均为堆，调整堆中元素，使以heap[top]为根结点的树为堆。
     * n为堆中元素总数。
     */
    int HeapAdjust(int *heap, int top, int n)
    {
        int j = 2 * top + 1;    //左孩子结点
        int temp = heap[top];

        while (j < n)
        {
            if (j + 1 < n&&heap[j + 1] < heap[j])
                j++;    //使j指向左右孩子中较小的结点。
            if (heap[j] >= temp)
                break;
            heap[top] = heap[j];
            top = j;
            j = 2 * top + 1;
        }
        heap[top] = temp;
        return 0;
    }

    /*
     * 堆排序算法。
     * 形参heap为大顶堆时，实现的是由小到大；
     * 形参heap为小顶堆时，实现的是由大到小；
     */
    int HeapSort(int *heap, int n)
    {
        int i;
        int temp;

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
    int CreatHeap(int *array, int n)
    {
        int i;
        //最后一个结点的编号为n-1，该结点的父节点(n-2)/2为最后一个非终端结点。
        //从结点(n-2)/2到根结点，依次进行堆调整。
        for (i = (n - 2) / 2; i >= 0; i--)
        {
            HeapAdjust(array, i, n);
        }
        return 0;
    }

}