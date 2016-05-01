Meteor.publish('getSimulationRuns', function() {
  return Simulations.find({ });
});
