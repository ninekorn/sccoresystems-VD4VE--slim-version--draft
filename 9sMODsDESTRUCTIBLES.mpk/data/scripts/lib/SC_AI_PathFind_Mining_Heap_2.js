using(console);

var temp;
var largest;
var left;
var right;
var swap;

//https://www.tutorialspoint.com/heap-sort-in-chash

var SC_AI_PathFind_Mining_Heap_2 = 
{
    heapSort: function (arr, n)
    {        
        for (var i = n / 2 - 1; i >= 0; i--)
        {
            i = Math.floor(i);
            heapify(arr, n, i);
        }
        for (var i = n - 1; i >= 0; i--)
        {
            i = Math.floor(i);
            var temp = arr[0];
            arr[0] = arr[i];
            arr[i] = temp;
            heapify(arr, i, 0);
        }
        return arr;
    }
};

function heapify(arr, n, i)
{
    largest = i;
    left = 2 * i + 1;
    right = 2 * i + 2;

	if (left < n && arr[left].fcost > arr[largest].fcost)
    {
        largest = left;
    }
    if (right < n && arr[right].fcost > arr[largest].fcost) 
	{
        largest = right;
    }

    if (largest != i)
    {
        swap = arr[i];
        arr[i] = arr[largest];
        arr[largest] = swap;
        heapify(arr, n, largest);
    }
}