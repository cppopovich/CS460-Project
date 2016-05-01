Session.set('dataSession',{map:"simple",algorithm:"naive",startLoc:1});

Template.homepage.events({
  'change #mapSelect': function(e,t) {
    e.preventDefault();
    var m = t.find('#mapSelect').value;
    var sObj = Session.get('dataSession');
    Session.set('dataSession', { map: m, algorithm: sObj.algorithm, startLoc: sObj.startLoc });
  },
  'change #selectAlgorithm': function(e,t) {
    e.preventDefault();
    var a = t.find('#selectAlgorithm').value;
    var sObj = Session.get('dataSession');
    Session.set('dataSession', { map: sObj.map, algorithm: a, startLoc: sObj.startLoc });
  },
  'change #selectStartNum': function(e,t) {
    e.preventDefault();
    var s = t.find('#selectStartNum').value;
    s = parseInt(s);
    var sObj = Session.get('dataSession');
    Session.set('dataSession', { map: sObj.map, algorithm: sObj.algorithm, startLoc: s });
    //console.log(s);
  }
});
