  
var myCanvas  = document.getElementById("myCanvas");
var myContext = myCanvas.getContext("2d");

var myClearColor    = '#d7cec7';
var myCurveColor    = '#565656';
var curveColor2     = '#56f256';
var curveColor3     = '#564356';
var myPointColor    = '#76323f';
var myEndPointColor = '#3232ff';

var myLineWidth = 3;
var UMBRAL_CALCULO_JACOBIANO = 1;
var DELTA = 0.001;

var L1 = 100;
var L2 = 100;
var L3 = 100;

//coordenadas iniciales de la base
var yInicio = 300;
var xInicio = 300;

//Coordenadas iniciales del target
var x  = 300;
var y  = 700;

//Coordenadas iniciales del p1
var x1 = 300;
var y1 = 400;

//Coordenadas del p2
var x2 = 300;
var y2 = 500;

//coordenadas iniciales del end effector
var x3 = 300;
var y3 = 600;

//variables globales necesarias
var invertido   = vec3.create();
var matriz      = mat3.create();
var target      = vec3.fromValues(x,y,0);
var orientacion = vec3.fromValues(0,0,0);
var transform   = mat3.fromValues(x1,y1,0,x2,y2,0,x3,y3,0);

//[x1,y1,0]
//[x2,y2,0]
//[ex,ey,0]
function dibujaBrazos(){
    
  //dibujado de las lineas
  // Enlaces
  //L1
  myContext.beginPath();  
  myContext.moveTo (xInicio, yInicio);
  myContext.lineTo (transform[0], transform[1]);
  myContext.lineWidth   = myLineWidth;
  myContext.strokeStyle = myCurveColor;
  myContext.stroke();
  
  //L2
  myContext.beginPath(); 
  myContext.moveTo (transform[0], transform[1]);
  myContext.lineTo (transform[3], transform[4]);
  myContext.lineWidth   = myLineWidth;
  myContext.strokeStyle = curveColor2;
  myContext.stroke();
  
  
  //L3
  myContext.beginPath(); 
  myContext.moveTo (transform[3], transform[4]);
  myContext.lineTo (transform[6], transform[7]);
  myContext.strokeStyle = curveColor3;
  myContext.stroke();


  //Uniones
  //Inicio
  myContext.beginPath();
  myContext.arc(xInicio, yInicio, 10, 0, 2 * Math.PI, false);
  myContext.fillStyle   = myPointColor;
  myContext.fill();
  myContext.strokeStyle = myPointColor;
  myContext.stroke();
    
  //Esto es para dibujado de puntos
  //Codo
  myContext.beginPath();
  //dinuja el circulo
  myContext.arc(transform[0] , transform[1], 10, 0, 2 * Math.PI, false);
  myContext.fillStyle   = myPointColor;
  myContext.fill();
  myContext.strokeStyle = myPointColor;
  myContext.stroke();

  //Punto final
  myContext.beginPath();
  myContext.arc(transform[3], transform[4], 10, 0, 2 * Math.PI, false);
  myContext.fillStyle   = myPointColor;
  myContext.fill();
  myContext.strokeStyle = myPointColor;
  myContext.stroke();
    
  //Punto a alcanzar
  myContext.beginPath();
  myContext.lineWidth   = 1;
  myContext.arc(transform[6], transform[7], 8, 0, 2 * Math.PI, false);
  myContext.fillStyle   = myPointColor;
  myContext.fill();
  myContext.strokeStyle = myEndPointColor;
  myContext.stroke();
   
  //Punto a alcanzar
  myContext.beginPath();
  myContext.lineWidth   = 1;
  myContext.arc(x,y, 8, 0, 2 * Math.PI, false);
  myContext.fillStyle   = myPointColor;
  myContext.fill();
  myContext.strokeStyle = myEndPointColor;
  myContext.stroke(); 

  myContext.beginPath();
  myContext.lineWidth   = 1;
  myContext.arc(xInicio,yInicio,(L1 + L2 + L3) , 0, 2 * Math.PI, false);
  myContext.strokeStyle = myEndPointColor;
  myContext.stroke(); 
  
}

//[x1,y1,0]
//[x2,y2,0]
//[ex,ey,0]
function calcularJacobiano(){
  
  var eje = vec3.fromValues(0,0,1);
  var e   = vec3.fromValues(transform[6],transform[7],0);

  var vector0 = vec3.create();
  vec3.cross(vector0,eje, e);
    
  var vector1 = vec3.create();
  var codoaEndEffectorVector = vec3.create();
  vec3.sub(codoaEndEffectorVector, e, vec3.fromValues(transform[0], transform[1], 0));
  vec3.cross(vector1, eje, codoaEndEffectorVector);
    
  var vector2 = vec3.create();
  var codo2aEndEffectorVector = vec3.create();
  vec3.sub(codo2aEndEffectorVector,e,vec3.fromValues(transform[3], transform[4],0));
  vec3.cross(vector2,eje,codo2aEndEffectorVector);

  mat3.set(matriz,
             vector0[0],vector1[0],vector2[0],
             vector0[1],vector1[1],vector2[1],
             vector0[2],vector1[2],vector2[2]);
  
}

function borraCanvas() {
  
  myContext.fillStyle = myClearColor;
  myContext.fillRect (0, 0, myCanvas.width, myCanvas.height);
  
}

function applyDeltaOrientation(deltaOrientation){
  
  var signo = 1;
  
  if (target[1] > 300)
    signo = -1;
  
  orientacion[0] += (deltaOrientation[0]) * signo * DELTA;
  orientacion[1] += (deltaOrientation[1]) * signo * DELTA;
  orientacion[2] += (deltaOrientation[2]) * signo * DELTA;
  
  transform[0] = L1 * Math.cos(orientacion[0]) + xInicio;
  transform[1] = L1 * Math.sin(orientacion[0]) + yInicio;

  transform[3] = L2 * Math.cos(orientacion[0] - orientacion[1]) + transform[0];
  transform[4] = L2 * Math.sin(orientacion[0] - orientacion[1]) + transform[1];
  
  transform[6] = L3 * Math.cos((orientacion[1] + orientacion[0]) - orientacion[2]) + transform[3];
  transform[7] = L3 * Math.sin((orientacion[1] + orientacion[0]) - orientacion[2]) + transform[4];
  
}

// function isAllowed(x,y) {
//   
//   var max = L1 + L2 + L3 + yInicio;  
//   var min = Math.max (0, L1 - L2 - L3);
//   var d   = Math.sqrt (Math.pow(x, 2) + Math.pow(y, 2));
//   
//   return (d <= max) && (d >= min); 
//   
// }

function drawScene() {
  
  var efactor = vec3.fromValues(transform[6],transform[7],0);

  if (Math.abs(vec3.distance(vec3.fromValues(xInicio,yInicio,0),vec3.fromValues(x,y,0))) < L1 + L2 + L3) {
    
    if(Math.abs(vec3.distance(target,efactor)) > UMBRAL_CALCULO_JACOBIANO) {
      
      calcularJacobiano();
      
      var inverse = mat3.create();
      var v = vec3.create();
      var deltaOrientation = vec3.create();

      mat3.invert(inverse,matriz);
      vec3.subtract(v,target,efactor);
      vec3.transformMat3(deltaOrientation,v,inverse);
      applyDeltaOrientation(deltaOrientation);
      
    }
  }
  
  borraCanvas ();
  dibujaBrazos();

  requestAnimationFrame(drawScene);
  
}

function getMousePos(canvas, evt) {
  
  var rect = canvas.getBoundingClientRect();
  
  return {
    x: evt.clientX - rect.left,
    y: evt.clientY - rect.top
  }
  
}

function initGetPosition() {
  
  myCanvas.addEventListener('mousedown', function(evt) {
    
    var mousePos = getMousePos(myCanvas, evt);
    
    //if(isAllowed(mousePos.x,mousePos.y)) {
      x = mousePos.x;
      y = mousePos.y;
      target[0] = x;
      target[1] = y;
      target[2] = 0;
    //}
    
  }, false);
  
}

function initVector() {
  
  initGetPosition();  
  drawScene();
  
}

initVector();
