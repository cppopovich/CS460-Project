Template.homepage.helpers({
  getSimData: () => {
    //retrieving them all for now
    return Simulations.find({});
  }
});
