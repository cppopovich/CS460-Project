Template.chart_diagram.rendered = function() {
  //this way we don't interfere with blaze (Meteor's template engine)

  var margin = { top: 20, right: 20, bottom: 30, left: 40 },
  width = 960 - margin.left - margin.right, //960-60 = 900 is the width since we have margins
  height = 500 - margin.top - margin.bottom; //500 - 50 = 450 is the height with the top margins

  //going to want to set domain of x and y here (for now it will be time)

  //ordinal creates empty domain and an empty range

  var listOfTimes = Simulations.find({ node_name: 't' },{ time: 1, _id: 0 }).fetch(); //time and no _id will not be grabbed.  Also getting Array of times

  var x = d3.scale.ordinal()
  .domain(listOfTimes)
  .rangeRoundBands([0,width],.1); //First paremeter is the Interval and Second parmeter is the padding

  var distanceFromTarget = null; //Going to want to do some calculations to distance of each robot from the target at each interval (Maybe have the distance for each run be stored)

  //Linear constructs a new linear scale defaults to 0 and 1
  var y = d3.scale.linear()
  .domain(distanceFromTarget)
  .range([height, 0]); //to-do

}
