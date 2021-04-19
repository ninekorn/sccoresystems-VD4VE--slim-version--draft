using(npc);
using(console);
using(player);
using(ship);
using(game);
using(generator);
using(storage);
using(station);
using(items);
using(timer);
using(server);

function OnAIInited() 
{
    npc.SetDecisionsPerSecond(SHIP_ID, 11);
}

var player_id;
var initStartingStuff;
var droneIndex;
var playerName;

function Decision(args)
{
    npc.InstantStop(SHIP_ID);
}

function OnDied(args)
{

}

function OnTakeDamage(args) {
    //NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}

function OnFrame(args) {

}
/*function OnTakeDamage(args)
{
    NpcLib.StandardOnTakeDamage(SHIP_ID, args);
}*/

function OnUpdateCache(args) {

}

/*function OnCalculateShipParameters(args)
{
    console.Print("OnCalculateShipParameters");
}

function OnCalculateInventoryCache(args) {
    console.Print("OnCalculateInventoryCache");
}

function OnCalculateSkillsCache(args) {
    console.Print("OnCalculateSkillsCache");
}

function OnCalculateBuffsCache(args) {
    console.Print("OnCalculateBuffsCache");
}
*/

/*function OnDied(args) {

}

function OnFrame(args)
{

}

function OnFinished(args) {

}

function OnCancel(args) {

}

function OnTakeDamage(args) {

}*/