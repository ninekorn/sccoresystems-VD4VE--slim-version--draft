using(console);


var SC_AI_PathFind_Mining_NodeDistance_2 =
{
    //sebastian lague pathfinding tutorial - i already explained on the forums that i was mostly inspired by sebastian lagues pathfinding tutorial and this function is a translation of his youtube free pathfinding tutorial.
    checkNodeDistance: function (nodeA, nodeB)
    {
        var dstX = Math.abs((nodeA.x) - (nodeB.x));
        var dstZ = Math.abs((nodeA.y) - (nodeB.y));

        if (dstX > dstZ)
            return 14 * dstZ + 10 * (dstX - dstZ);
        return 14 * dstX + 10 * (dstZ - dstX);
    }
};





























