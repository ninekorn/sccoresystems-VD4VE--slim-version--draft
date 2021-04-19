using(console);

var temp;
//https://www.tutorialspoint.com/heap-sort-in-chash

var SC_Pathfind_Heap_cc_5 = {

    heapSort: function (arr, n, typeOfSort)
    {
        
        for (var i = n / 2 - 1; i >= 0; i--)
        {
            i = Math.floor(i);
            heapify(arr, n, i, typeOfSort);
            //console.PrintError(i);
        }
        for (var i = n - 1; i >= 0; i--)
        {
            i = Math.floor(i);
            //console.PrintError(i);
            var temp = arr[0];
            arr[0] = arr[i];
            arr[i] = temp;
            heapify(arr, i, 0, typeOfSort);
        }
        return arr;
    }
};

var largest;
var left;
var right;
var swap;

function heapify(arr, n, i, typeOfSort)
{
    largest = i;
    left = 2 * i + 1;
    right = 2 * i + 2;

    if (typeOfSort == "fcost")
    {
        if (left < n && arr[left].fcost > arr[largest].fcost)
        {
            largest = left;
        }
        if (right < n && arr[right].fcost > arr[largest].fcost) {
            largest = right;
        }

        /*if (left > n && arr[left].gcost > arr[largest].gcost) {
            largest = left;
        }
        */
        //works
        /*if (right > n && arr[right].gcost > arr[largest].gcost) {
            largest = right;
        }*/
        //works



        /*if (left < n && arr[left].gcost > arr[largest].gcost) {
            largest = left;
        }
        if (right < n && arr[right].gcost > arr[largest].gcost) {
            largest = right;
        }*/
    }
 
    if (largest != i)
    {
        swap = arr[i];
        arr[i] = arr[largest];
        arr[largest] = swap;
        heapify(arr, n, largest);
    }
}




//works
/*if (left < n && arr[left].fcost > arr[largest].fcost) {
    largest = left;
}
if (right < n && arr[right].fcost > arr[largest].fcost) {
    largest = right;
}

if (left < n && arr[left].gcost > arr[largest].gcost) {
    largest = left;
}

if (right > n && arr[right].gcost > arr[largest].gcost) {
    largest = right;
}*/
//works