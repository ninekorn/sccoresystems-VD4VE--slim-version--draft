
using(console);
using(game);

include(SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.js);
include(SC_AI_Drone_Combat_cc_Rout_Com_Att_3.js);
include(SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js);

var tempLceData = null;
var dataToReturn = null;

//game.ShipPlaySound(args.ship_id, "SC_NPC/SC_NPC_Combat_TalkDevice_TargetAcquired.ogg", 2, false);

var lockDist = 25;


var SC_AI_Drone_Combat_cc_Rout_Com_3 =
{
    AICombatRoutine: function (cData, ceData) {



        /*var otherShipCoord = ship.GetCoordinates(otherShip);
        var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);
        var distNpcToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);

        if (distPlayerToEnemy <= lockdist && distNpcToEnemy <= lockdist)   //&& distPlayerToDrone <= lockdist// && distToEnemy <= distPlayerToDrone
        {
            ship.SetDamageByShip(otherShip, 1, ceData.objt.nid);
            ship.SetDamageByShip(ceData.objt.nid, 1, otherShip);


            //var cdamage = ship.SetDamageByShip(otherShip.id, 1, player_id);          
            //var gdamage = ship.GetDamageByShip(ceData.objt.pid, otherShip);

            //var dist = ship.GetDistanceToObj(ceData.objt.nid, otherShip);
            var someData = { id: otherShip, dist: distNpcToEnemy, eToD: 0 };
            shipNDist.push(someData);
        }*/

        //ceData.lLSwtc = cData.lLSwtc;

        //console.PrintError("ceData.lLSwtc " + ceData.lLSwtc);
        if (cData.lLSwtcCounter >= 0) // 5 frames before checking for targets or unwalkable tiles. //19*10 was a bit not enough
        {
            if (ceData.lLSwtc == 0) // main check for find target. if no target, this script will not be used.
            {
                //console.PrintError("target/unwalkables check");
                cData.cdtSwtc = 1;
                cData.cdSwtc = 1;
                tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.npcFindTarget(ceData);
                ceData = tempLceData;
                //cData.lLSwtc = 1;
                //ceData.lLSwtc = 1;
            }
            else {
                //tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.npcFindTarget(ceData);
                //ceData = tempLceData;
            }
            cData.lLSwtcCounter = 0;
        }
       
        cData.lLSwtcCounter++;


        if (ceData.hasUnwalkableTiles == 1)
        {
            

            //cData.hasUnwalkableTilesArray = null;
            //cData.hasUnwalkableTilesArray = [];

            for (var i = 0; i < ceData.hasUnwalkableTilesArray.length;i++) {
                
                cData.hasUnwalkableTilesArray.push({ id: ceData.hasUnwalkableTilesArray[i].id, coords: ceData.hasUnwalkableTilesArray[i].coords, class: ceData.hasUnwalkableTilesArray[i].class });
            }
            //console.PrintError("has unwalkable tiles");
            //cData.hasUnwalkableTilesArray = ceData.hasUnwalkableTilesArray;
            cData.hasUnwalkableTiles = 1;
            ceData.hasUnwalkableTiles = -1;

        }


        //console.PrintError("4what is going on");

        if (ceData.ts != -1) // target ship.
        {
            if (generator.ShipExists(ceData.ts)) {
                var otherShipCoord = ship.GetCoordinates(ceData.ts);

                var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);

                if (distPlayerToEnemy <= lockDist) // && distPlayerToDrone <= 20
                {
                    var distDroneToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);

                    if (distDroneToEnemy <= lockDist)
                    {

                    }
                    else {
                        ceData.lLSwtc = 0;
                        ceData.ts = -1;
                    }
                }
                else {
                    ceData.lLSwtc = 0;
                    ceData.ts = -1;
                }

            }
            else {

            }
        }
        //console.PrintError("5what is going on");
        /*if (ceData.eneL > 15) // distance check
        {
            

            ceData.lLSwtc = 0;
            ceData.ts = -1;
        }*/
        //console.PrintError("nid: " + cData.objt.nid);
        if (ceData.ts != -1) // target ship.
        {
            //console.PrintError("npc has a target ceData.ts != -1");
            if (!generator.ShipExists(ceData.ts)) {

                if (ship.GetCurrentValue(ceData.ts, "structure") <= 0) {
                    if (npc.IsFriendlyLocked(ceData.objt.nid)) // if drone was friendly locked on it, just make sure to remove the lock although i have never tested IF the lock is auto removed which i think it is.
                    {
                        npc.Unlock(ceData.objt.nid);
                    }

                    if (npc.IsLocked(ceData.objt.nid)) // if drone was locked on it, just make sure to remove the lock although i have never tested IF the lock is auto removed which i think it is.
                    {
                        npc.Unlock(ceData.objt.nid);
                    }

                    ceData.lLSwtc = 0;
                    ceData.ts = -1;
                }
                
            }
            else {
                if (ship.GetCurrentValue(ceData.ts, "structure") > 0)
                {
                    if (ceData.tslc >= lockDist) // checking every lockDist frames if the enemy is still alive.
                    {
                        if (ship.GetCurrentValue(ceData.ts, "structure") >= 0) {
                            game.ShipPlaySound(ceData.objt.nid, "special/TargetAcquired.ogg", 2, false);
                        }
                        ceData.tsl = -3;
                        ceData.tslc = 0;
                    }

                    if (ceData.tsl == -3) {
                        if (ceData.tslc >= 3) // checking every lockDist frames if the enemy is still alive.
                        {
                            game.ShipPlaySound(ceData.objt.nid, "special/TargetDestroyed.ogg", 2, false);
                            ceData.tslc = 0;
                        }
                    }
                    ceData.tslc++;



                    if (npc.IsLocked(ceData.objt.nid)) {
                        console.PrintError("is locked");
                        /*//TO READD LATER
                        //TO READD LATER
                        //TO READD LATER
                        tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_3.npcCombatRoutine(ceData);
                        ceData = tempLceData;
                        //TO READD LATER
                        //TO READD LATER
                        //TO READD LATER
    
                        //npc.Unlock(ceData.objt.nid);*/
                    }
                    else {
                        console.PrintError("is !locked");

                        if (generator.ShipExists(ceData.ts) && ship.GetCurrentValue(ceData.ts, "structure") > 0) {
                            console.PrintError("test3");
                            var otherShipCoord = ship.GetCoordinates(ceData.ts);

                            var distPlayerToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.pData.pCoord);

                            if (distPlayerToEnemy <= lockDist) // && distPlayerToDrone <= 20
                            {
                                var distDroneToEnemy = SC_Utilities.GetDistance(otherShipCoord, ceData.nData.nCoord);

                                if (distDroneToEnemy <= lockDist) {
                                    console.PrintError("test2");
                                    //cData.cFWP = otherShipCoord;
                                    //cData.cFWPD = ceData.eneL;
                                    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);



                                }
                                else {
                                    console.PrintError("test1");
                                    if (cData.objt.command == 0 && ceData.ts == -1) {
                                        cData.mSwtc = 1;
                                        cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);
                                    }
                                    else if (cData.objt.command == 1 && ceData.ts == -1) {
                                        //console.PrintError("test");
                                        /*if (cData.mSwtc == 0) {
                    
                                            npc.InstantStop(cData.objt.nid);
                    
                                            cData.mSwtc = 1;
                                        }*/

                                        cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);


                                        //ADD SCRIPTS FOR THE PERCEPTRON, TO GET THE NEXT NODE OUT OF THE ABOVE FUNCTION, REMOVE any sorts or form of npc.InstantStop or npc.Stop or npc.StickToPoint. create scripts and have the perceptron start a new script hierarchy otherwise the SC_PathFind_cc_3.js will be cluttered and bigger than 20-30 kb.
                                    }
                                }
                            }
                            else {
                                console.PrintError("test0");
                                if (cData.objt.command == 0 && ceData.ts == -1) {
                                    console.PrintError("test00");
                                    cData.mSwtc = 1;
                                    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);
                                }
                                else if (cData.objt.command == 1 && ceData.ts == -1) {
                                    console.PrintError("test01");

                                    /*if (cData.mSwtc == 0) {
    
                                        //npc.InstantStop(cData.objt.nid);
    
                                        cData.mSwtc = 1;
                                    }*/
                                    //console.PrintError("test");
                                    /*if (cData.mSwtc == 0) {
                
                                        npc.InstantStop(cData.objt.nid);
                
                                        cData.mSwtc = 1;
                                    }*/

                                    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);


                                    //ADD SCRIPTS FOR THE PERCEPTRON, TO GET THE NEXT NODE OUT OF THE ABOVE FUNCTION, REMOVE any sorts or form of npc.InstantStop or npc.Stop or npc.StickToPoint. create scripts and have the perceptron start a new script hierarchy otherwise the SC_PathFind_cc_3.js will be cluttered and bigger than 20-30 kb.
                                }
                            }
                        }
                    }
                }
            }
            

            ceData.tslc++;
        }
        else {


           // console.PrintError("npc doesn't have a target ceData.ts != -1");

            //console.PrintError("is the frame coming here " + cData.swtchForPathfind);

            if (cData.swtchForPathfind == 0)
            {
               // console.PrintError("is the frame coming here " + cData.swtchForPathfind + " cData.objt.command " + cData.objt.command + " ceData.ts " + ceData.ts );
                //cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);

              
                if (cData.objt.command == 0 && ceData.ts == -1) {
                    //console.PrintError("npcStayInFormation");
                    cData.mSwtc = 1;
                    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);
                }
                else if (cData.objt.command == 1 && ceData.ts == -1) {
                    //console.PrintError("npcPathfindToWaypoint");
                    //console.PrintError("test");
                    /*if (cData.mSwtc == 0) {

                        npc.InstantStop(cData.objt.nid);

                        cData.mSwtc = 1;
                    }*/

                    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);


                    //ADD SCRIPTS FOR THE PERCEPTRON, TO GET THE NEXT NODE OUT OF THE ABOVE FUNCTION, REMOVE any sorts or form of npc.InstantStop or npc.Stop or npc.StickToPoint. create scripts and have the perceptron start a new script hierarchy otherwise the SC_PathFind_cc_3.js will be cluttered and bigger than 20-30 kb.
                }
                else {
                    console.PrintError("command: " + cData.objt.command + " ceData.ts " + ceData.ts);
                    console.PrintError("02is the frame coming here ");
                }
            }
            
        }

        cData.cLFWP = cData.cFWP;
        cData.cLFWPD = cData.cFWPD;

        dataToReturn = { forData: cData, comData: ceData };
        return dataToReturn;
    }
};







/*if(ceData.tsl == -2)
{
    if (generator.ShipExists(ceData.ts) && ship.GetCurrentValue(ceData.ts, "structure") >= 0)
    {
        game.ShipPlaySound(ceData.objt.nid, "TargetAcquired.ogg", 2, false);
    }
    else
    {
        game.ShipPlaySound(ceData.objt.nid, "TargetDestroyed.ogg", 2, false);
    }
    ceData.tsl = -3;
}


if(ceData.tsl == -3)
{
    if(ceData.tslc >= lockDist) // checking every lockDist frames if the enemy is still alive.
    {
        if (generator.ShipExists(ceData.ts) || ship.GetCurrentValue(ceData.ts, "structure") <= 0)
        {
            game.ShipPlaySound(ceData.objt.nid, "TargetDestroyed.ogg", 2, false);
            ceData.lLSwtc = 0;
        }
        ceData.tslc = 0;
    }
}*/




//using(console);
//using(generator);
//using(ship);


//include(SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.js);
//include(SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.js);
//include(SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.js);
//include(SC_AI_Drone_Combat_cc_Rout_Com_Att_3.js);
/*
var tempLceData = null;
var dataToReturn = null;

var SC_AI_Drone_Combat_cc_Rout_Com_3 =
{
    AICombatRoutine: function (cData, ceData)
    {       
        //console.PrintError("cData.objt.command : " + cData.objt.command + " mSwtc: " + cData.mSwtc + " ceData.ts: " + ceData.ts + " cData.sSwtc: " + cData.sSwtc + " ceData.lLSwtc: " + ceData.lLSwtc);

        //if (ceData.lLSwtc == 0 || ceData.ts == -1)
        //{
        //    tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.npcFindTarget(ceData);
        //    ceData = tempLceData;
        //}
        //if (cData.objt.command == 1 && ceData.ts == -1)
        //{
        //    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcStayInFormationPathfind(cData);
        //}





        //sending drone to the station immediately  when it is damaged.
        //sending drone to the station immediately  when it is damaged.
        //if (ceData.ts == -1)
        //{
        //    ceData.lLSwtc = 0;
        //}
        //else
        //{
        //    if (generator.ShipExists(ceData.ts))
        //    {
        //        if (ship.GetCurrentValue(ceData.ts, "structure") <= 0)
        //        {
        //            ceData.ts = -1;
         //           ceData.lLSwtc = 0;
         //       }
         //       else
       //         {
         //           ceData.lLSwtc = 1;
        //        }
        //    }
        //    else
         //   {
         //       if (npc.IsLocked(ceData.objt.nid) || npc.IsFriendlyLocked(ceData.objt.nid)) {
         //           npc.Unlock(ceData.objt.nid);
        //        }
        //        ceData.ts = -1;
         //       ceData.lLSwtc = 0;
        //    }
        //}
         //sending drone to the station immediately  when it is damaged.
         //sending drone to the station immediately  when it is damaged.


        //in order to put back the drone in formation with the if else statements below. see comments index 0
        //cData.mSwtc = 1;
        //cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);





        //if (ceData.lLSwtc == 0 || ceData.ts == -1)
        //{
        //    tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.npcFindTarget(ceData);
        //    ceData = tempLceData;
        //}

        //console.PrintError("id: " + cData.objt.nid +" targ: " + ceData.ts);

        //if (ceData.ts != -1) {
        //    cData.mSwtc = 1;
        //    cData.fSwtc = 1;
        //    ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_3.npcCombatRoutine(ceData);
        //}
        //else if (cData.objt.command == 0 && ceData.ts == -1 || cData.objt.command == 1 && ceData.ts == -1) //index 0 comments: getting the ship back in formation conditional statements.
        //{

        //    if (cData.mSwtc != 3 && cData.mSwtc != 4 && cData.mSwtc != 5 && cData.mSwtc != 6 && cData.mSwtc != 7) {
         //       cData.mSwtc = 1;
         //   }
         //if (cData.objt.bidSwtch == 1)
         //   {
          //      console.PrintError("is in base vicinity");
         //       //cData[currentObjective.droneIndex].nData = nData;
         //       //cData[currentObjective.droneIndex].pData = pData;
         //       //cData[currentObjective.droneIndex].objt = currentObjective;
         //       //cData[currentObjective.droneIndex].cFWP = currentWaypoint;
         //       //cData[currentObjective.droneIndex].cFWPD = distToWaypoint;
         //       //cData.sSwtc = 2;
         //       cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData, cData.objt.bid);
          //  }
          //  else {
//
         //       console.PrintError("is not in base vicinity");
         //       cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);
         //   }
        //}
        //else {
        //    console.PrintError("is not the other choices");
        //}
        //else if (cData.objt.command == 1 && ceData.ts == -1)
        //{
        //    if (cData.mSwtc == 0) {
        //        npc.InstantStop(cData.objt.nid);
        //        cData.mSwtc = 1;
        //    }
        //    cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcStayInFormationPathfind(cData);
        //}




        //GET COMBAT TARGET 
        //if (ceData.lLSwtc == 0 || ceData.ts == -1)
        //{
        //    tempLceData = SC_AI_Drone_Combat_cc_Rout_Com_FndTarg_3.npcFindTarget(ceData);
        //    ceData = tempLceData;
        //}
        //GET COMBAT TARGET 




        if (ceData.ts != -1)
        {
            cData.mSwtc = 1;
            cData.fSwtc = 1;
            ceData = SC_AI_Drone_Combat_cc_Rout_Com_Att_3.npcCombatRoutine(ceData);


        }
        else
        {
            //if (cData.fSwtc == 1) 
            //{
            //}



            if (cData.objt.command == 0 && ceData.ts == -1)
            {
                cData.mSwtc = 1;

                cData = SC_AI_Drone_Combat_cc_Rout_For_SFWP_3.npcStayInFormation(cData);
            }
            else if (cData.objt.command == 1 && ceData.ts == -1)
            {
                //console.PrintError("test");
                if (cData.mSwtc == 0)
                {
                    npc.InstantStop(cData.objt.nid);
                    cData.mSwtc = 1;
                }

                cData = SC_AI_Drone_Combat_cc_Rout_For_SFWPP_3.npcPathfindToWaypoint(cData);


                //ADD SCRIPTS FOR THE PERCEPTRON, TO GET THE NEXT NODE OUT OF THE ABOVE FUNCTION, REMOVE any sorts or form of npc.InstantStop or npc.Stop or npc.StickToPoint. create scripts and have the perceptron start a new script hierarchy otherwise the SC_PathFind_cc_3.js will be cluttered and bigger than 20-30 kb.
            }
        }

        dataToReturn = { forData: cData, comData: ceData };

        return dataToReturn;
    }
};*/
