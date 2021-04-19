using(console);
include(SC_AI_PathFind_Mining_GetGridWorldPos_3.js);

var SC_AI_PathFind_Mining_GetGridIndex_3 =
{
    getNewGridIndex: function (initialPos, seekerPos, gridWorldSize, sid) 
    {
        var gridData = SC_AI_PathFind_Mining_GetGridWorldPos_3.getGridWorldPosition(initialPos, seekerPos, gridWorldSize, sid);

        var testX = 0;
        var testY = 0;
        var minY = 0;
        var maxY = 0;

        var minX = 0;
        var maxX = 0;

        var index = 0;

        if (gridData.x < initialPos.x) {
            testX = -gridData.gridX;
        }
        else {
            testX = gridData.gridX;
        }

        if (gridData.y < initialPos.y) {
            testY = -gridData.gridY;
        }
        else {
            testY = gridData.gridY;
        }

        var someAdder = 0;

        if (gridData.gridX >= gridData.gridY)
        {
            if (testX <= 0)
            {
                minX = gridData.gridX;
                maxX = gridData.gridX;

                maxY = maxX;
                minY = maxY-1;
              

                var currentContainedGrids = (minX + maxX) * (minY + maxY + 1);

                if (testY >= 0) 
                {
                    someAdder = (minY + gridData.gridY + 1); 
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder;

                    someAdder = (minY + gridData.gridY + 1); 
                    var tot = minY + maxY + 1; 
                    index = currentContainedGrids - (tot) + someAdder; 

                }
                else
                {
                    someAdder = (minY - gridData.gridY + 1);

                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder;
                }
            }
            else
            {
                minX = gridData.gridX; 
                maxX = gridData.gridX; 

                minY = minX; 
                maxY = maxX;

                var currentContainedGrids = (minX + maxX+1) * (minY + maxY);

                if (testY >= 0)
                {
                    someAdder = (minY + gridData.gridY + 1);
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids + (tot) - someAdder;
                }
                else
                {
                    someAdder = (minY - gridData.gridY + 1); 
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids + (tot) - someAdder; 
                }
            }
        }
        else if (gridData.gridX < gridData.gridY)
        {
            if (testY <= 0) 
            {
                minY = gridData.gridY;
                maxY = gridData.gridY;

                minX = minY;
                maxX = minX-1;

                var currentContainedGrids = (minX + maxX) * (minY + maxY);

                var someAdder = 0;
                if (testX >= 0)
                {
                    someAdder = (minX - gridData.gridX + 1);
                    var tot = minY + maxY + 1;
                    index = currentContainedGrids - (tot) + someAdder; 
                }
                else
                {
                    someAdder = (minX + gridData.gridX + 1);
                    var tot = minY + maxY + 1; 
                    index = currentContainedGrids - (tot) + someAdder;
                }
            }
            else
            {
                minY = gridData.gridY; 
                maxY = gridData.gridY; 

                minX = minY; 
                maxX = maxY; 

                var currentContainedGrids = (minX + maxX) * (minY + maxY); 

                var someAdder = 0;
                if (testX >= 0) {
 
                    someAdder = (minX + gridData.gridX); 
                    index = currentContainedGrids + someAdder; 
                }
                else {
       
                    someAdder = (maxX - gridData.gridX);
                    index = currentContainedGrids + someAdder;
                }
            }
        }
        return { gridData: gridData, index: index };
    }
};





























