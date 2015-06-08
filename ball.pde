// Kinect Physics Example by Amnon Owed (15/09/12)

//edited by Arindam Sen
 
// import libraries
import processing.opengl.*; // opengl
import SimpleOpenNI.*; // kinect
import blobDetection.*; // blobs
import toxi.geom.*; // toxiclibs shapes and vectors
import toxi.processing.*; // toxiclibs display
import shiffman.box2d.*; // shiffman's jbox2d helper library
import org.jbox2d.collision.shapes.*; // jbox2d
import org.jbox2d.dynamics.joints.*;
import org.jbox2d.common.*; // jbox2d
import org.jbox2d.dynamics.*; // jbox2d

// declare SimpleOpenNI object
SimpleOpenNI context;
// declare BlobDetection object
BlobDetection theBlobDetection;
// ToxiclibsSupport for displaying polygons
ToxiclibsSupport gfx;
// declare custom PolygonBlob object (see class for more info)
PolygonBlob poly;
 
// PImage to hold incoming imagery and smaller one for blob detection
PImage blobs;
// the kinect's dimensions to be used later on for calculations
int kinectWidth = 640;
int kinectHeight = 480;
PImage cam = createImage(640, 480, RGB);

int gravity = -40;
int wind = 0;

int colorChannel = 0;

// to center and rescale from 640x480 to higher custom resolutions
float reScale;
 
// background and blob color
color bgColor, blobColor;
// three color palettes (artifact from me storingmany interesting color palettes as strings in an external data file ;-)
String[] palettes = {
  "-0,-1,-45678,-7890,-35467,-68457,-6457,-1234,-4356,-3456",
  "-1,-0,-3456,-8790435,-34356,-6589,-4578,-23425,-978675,-3421342",
  "-45678456, -435345,-56756,-34563,-43523673,-5461234,-4578,-6756,-45,-7458", 
  "-78926, -453457,-35354,-674,-534,-634,-43234,-6556,-22324,-532", 
};
color[] colorPalette;
 
// the main PBox2D object in which all the physics-based stuff is happening
Box2DProcessing box2d;
// list to hold all the custom shapes (circles, polygons)
ArrayList<CustomShape> polygons = new ArrayList<CustomShape>();
 
void setup() {
  println("SET UP");
  // it's possible to customize this, for example 1920x1080
  size(1280, 800);
  context = new SimpleOpenNI(this);
  // initialize SimpleOpenNI object
  if (!context.enableScene()) { 
    // if context.enableScene() returns false
    // then the Kinect is not working correctly
    // make sure the green light is blinking
    println("Kinect not connected!"); 
    exit();
  } else {
    // mirror the image to be more intuitive
    context.setMirror(true);
    // calculate the reScale value
    // currently it's rescaled to fill the complete width (cuts of top-bottom)
    // it's also possible to fill the complete height (leaves empty sides)
    reScale = (float) width / kinectWidth;
    // create a smaller blob image for speed and efficiency
    blobs = createImage(kinectWidth/3, kinectHeight/3, RGB);
    // initialize blob detection object to the blob image dimensions
    theBlobDetection = new BlobDetection(blobs.width, blobs.height);
    theBlobDetection.setThreshold(0.3);
    // initialize ToxiclibsSupport object
    gfx = new ToxiclibsSupport(this);
    // setup box2d, create world, set gravity
    box2d = new Box2DProcessing(this);
    box2d.createWorld();
    box2d.setGravity(wind, gravity);
    // set random colors (background, blob)
    setRandomColors(1);
    
    float gap = kinectWidth / 21;
    for (int i=0; i<20; i++)
    {
      drawString(gap * (i+1), 2, 10);
    }
  }
}

void drawString(float x, float size, int cards) {

  float gap = kinectHeight/cards;
  // anchor card
  CustomShape s1 = new CustomShape(x, -40, size, BodyType.DYNAMIC);
  polygons.add(s1);
  
  CustomShape last_shape = s1;
  CustomShape next_shape;
  for (int i=0; i<cards; i++)
  {
    float y = -20 + gap * (i+1);
    next_shape = new CustomShape(x, -20 + gap * (i+1), size, BodyType.DYNAMIC);
    DistanceJointDef jd = new DistanceJointDef();

    Vec2 c1 = last_shape.body.getWorldCenter();
    Vec2 c2 = next_shape.body.getWorldCenter();
  // offset the anchors so the cards hang vertically
    c1.y = c1.y + size / 5;
    c2.y = c2.y - size / 5;
    jd.initialize(last_shape.body, next_shape.body, c1, c2);
    jd.length = box2d.scalarPixelsToWorld(gap - 1);
    box2d.createJoint(jd);
    polygons.add(next_shape);
    last_shape = next_shape;
  }
}
 
void draw() {
  background(bgColor);
  // update the SimpleOpenNI object
  context.update();
  cam = context.sceneImage().get();
  
  cam.loadPixels();
  color black = color(0,0,0);
  // filter out grey pixels (mixed in depth image)
  for (int i=0; i<cam.pixels.length; i++)
  { 
    color pix = cam.pixels[i];
    int blue = pix & 0xff;
    if (blue == ((pix >> 8) & 0xff) && blue == ((pix >> 16) & 0xff))
    {
      cam.pixels[i] = black;
    }
  }
  cam.updatePixels();
  
  // copy the image into the smaller blob image
  blobs.copy(cam, 0, 0, cam.width, cam.height, 0, 0, blobs.width, blobs.height);
  // blur the blob image
  blobs.filter(BLUR, 1);
  // detect the blobs
  theBlobDetection.computeBlobs(blobs.pixels);
  // initialize a new polygon
  poly = new PolygonBlob();
  // create the polygon from the blobs (custom functionality, see class)
  poly.createPolygon();
  // create the box2d body from the polygon
  poly.createBody();
  // update and draw everything (see method)
  updateAndDrawBox2D();
  // destroy the person's body (important!)
  poly.destroyBody();
  // set the colors randomly every 240th frame
  //setRandomColors(240);
}
 
void updateAndDrawBox2D() {
  // if frameRate is sufficient, add a polygon and a circle with a random radius

  
  if (frameRate > 25) {
    //CustomShape shape1 = new CustomShape( kinectWidth/2, -50, -1,BodyType.DYNAMIC) ;
     CustomShape shape2 = new CustomShape(kinectWidth/2, -50, random(2.5, 10),BodyType.DYNAMIC);
    //polygons.add(shape1);
    polygons.add(shape2);
    
  }
  // take one step in the box2d physics world
  box2d.step();
 
  // center and reScale from Kinect to custom dimensions
  translate(0, (height-kinectHeight*reScale)/2);
  scale(reScale);
 
  // display the person's polygon  
  noStroke();
  fill(blobColor);
  gfx.polygon2D(poly);
 
  // display all the shapes (circles, polygons)
  // go backwards to allow removal of shapes
  for (int i=polygons.size()-1; i>=0; i--) {
    CustomShape cs = polygons.get(i);
    // if the shape is off-screen remove it (see class for more info)
    
    
    if (cs.done()) {
      polygons.remove(i);
    // otherwise update (keep shape outside person) and display (circle or polygon)
    } else {
      cs.update();
      cs.display();
    }
  }
}
 
// sets the colors every nth frame
void setRandomColors(int nthFrame) {
  //if (frameCount % nthFrame == 0) {
    // turn a palette into a series of strings
    String[] paletteStrings = split(palettes[int(random(palettes.length))], ",");
    // turn strings into colors
    colorPalette = new color[paletteStrings.length];
    for (int i=0; i<paletteStrings.length; i++) {
      colorPalette[i] = int(paletteStrings[i]);
    }
    // set background color to first color from palette
    bgColor = colorPalette[0];
    // set blob color to second color from palette
    blobColor = colorPalette[1];
    // set all shape colors randomly
    
    for (CustomShape cs: polygons) { cs.col = getRandomColor(); }
  //}
}
 
// returns a random color from the palette (excluding first aka background color)
color getRandomColor() {
  return colorPalette[int(random(1, colorPalette.length))];
}

void keyPressed()
{
   println("key");
  switch (key) {
    case 'a':
      wind -= 10;
      box2d.setGravity(wind, gravity);
      break;
    case 'd':
      wind += 10;
      box2d.setGravity(wind, gravity);
      break; 
    case 'w':
      gravity += 10;
      box2d.setGravity(wind, gravity);
      break;
    case 's':
      gravity -= 10;
      box2d.setGravity(wind, gravity);
      break;
    case 'g':
      gravity = 0;
      wind = 0;
      box2d.setGravity(wind, gravity);
      break;
    case 32:
      println("space");
      colorChannel++;
      setRandomColors(240);
      break;    
    default:  
      break;
  }
 
}
