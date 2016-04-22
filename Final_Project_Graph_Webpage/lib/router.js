Router.configure({
  notFoundTemplate: 'notFound',
  loadingTemplate: 'loading',
  layoutTemplate: 'layout'
});


Router.route('/', {
  layout: 'layout',
  template: "homepage",
  waitOn: function() {
    //going to subscribe to all data from this database for now
    Meteor.subscribe('getSimulationRuns');
  }
});
