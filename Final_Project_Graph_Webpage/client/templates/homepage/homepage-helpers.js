Template.homepage.helpers({
  getSimData: () => {
    //retrieving them all for now
    var data = Session.get('dataSession');
    return Simulations.find({map: data.map, type: data.algorithm,startno:data.startLoc}, { sort: { speed: 1 }});
    //return Simulations.find({ map: data.map, type: data.algorithm }, { sort: { startno: 1, speed: 1}});
  }
});
