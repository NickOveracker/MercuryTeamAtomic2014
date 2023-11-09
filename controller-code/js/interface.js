var viewController = function($scope, $interval, $http) {
   $interval(function() {
         $http.get('http://127.0.0.1:1992')
            .then(function(res) {
               $scope.data = res.data;
               $scope.data.servoTipX
                  = 270 - 120*Math.cos(Math.PI/180 * (10+res.data.servoPosition));
               $scope.data.servoTipY 
                  = 270 + -120*Math.sin(Math.PI/180 * (10+res.data.servoPosition));

            }, function(res) {
               console.log('error');
               console.log(res);
            });
      }, 50);
};
