#include "vtkActor.h"
#include "vtkCallbackCommand.h"
#include "vtkCamera.h"
#include "vtkJPEGReader.h"
#include "vtkLight.h"
#include "vtkOBJReader.h"
#include "vtkOSPRayLightNode.h"
#include "vtkOSPRayMaterialLibrary.h"
#include "vtkOSPRayPass.h"
#include "vtkOSPRayRendererNode.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
#include "vtkRendererCollection.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkSmartPointer.h"
#include "vtkTexture.h"

//----------------------------------------------------------------------------
class vtkProgressiveRenderLooper : public vtkCommand
{
  //a helper to automate progressive rendering
  public:
    vtkTypeMacro(vtkProgressiveRenderLooper, vtkCommand);

    static vtkProgressiveRenderLooper *New()
    {
      vtkProgressiveRenderLooper *self = new vtkProgressiveRenderLooper;
      self->RenderWindow = nullptr;
      self->ProgressiveCount = 0;
      return self;
    }

    void Execute(vtkObject *vtkNotUsed(caller),
                 unsigned long eventId,
                 void *vtkNotUsed(callData)) override
    {
      if (eventId == vtkCommand::TimerEvent)
      {
        if (this->RenderWindow)
        {
          vtkRenderer *renderer = this->RenderWindow->GetRenderers()->
            GetFirstRenderer();
          int maxframes = vtkOSPRayRendererNode::GetMaxFrames(renderer);
          if (this->ProgressiveCount < maxframes)
          {
            this->ProgressiveCount++;
            this->RenderWindow->Render();
            //cerr << this->ProgressiveCount << endl;
          }
        }
      } else {
        this->ProgressiveCount = 0;
      }
    }
    vtkRenderWindow *RenderWindow;
    int ProgressiveCount;
};

int main(int argc, char* argv[])
{
  vtkSmartPointer<vtkRenderWindow> renWin =
    vtkSmartPointer<vtkRenderWindow>::New();
  vtkSmartPointer<vtkRenderWindowInteractor> iren =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  iren->SetRenderWindow(renWin);
  vtkSmartPointer<vtkRenderer> renderer =
    vtkSmartPointer<vtkRenderer>::New();
  renWin->AddRenderer(renderer);
  renWin->SetSize(700,700);

  //announce that we will use OSPRay instead of OpenGL
  vtkSmartPointer<vtkOSPRayPass> ospray = vtkSmartPointer<vtkOSPRayPass>::New();
  renderer->SetPass(ospray);

  //ospray specific controls, for which there are no openGL equivalents
  //are handled like so
  vtkOSPRayRendererNode::SetRendererType("pathtracer", renderer);
  vtkOSPRayRendererNode::SetSamplesPerPixel(1, renderer);

  //use an environment map so that materials have something to reflect
  vtkSmartPointer<vtkTexture> textr = vtkSmartPointer<vtkTexture>::New();
  vtkSmartPointer<vtkJPEGReader> imgReader = vtkSmartPointer<vtkJPEGReader>::New();
  std::string fname = "tmp/wintersun.jpg";
  imgReader->SetFileName(fname.c_str());
  imgReader->Update();
  textr->SetInputConnection(imgReader->GetOutputPort(0));
  renderer->TexturedBackgroundOn();
  renderer->SetBackgroundTexture(textr);
  //orient the background environment
  double np[3] = {0,0,1};
  vtkOSPRayRendererNode::SetNorthPole(np, renderer);
  double ep[3] = {0,1,0};
  vtkOSPRayRendererNode::SetEastPole(ep, renderer);

  vtkSmartPointer<vtkActor> actor;
  vtkProperty *prop;
  vtkSmartPointer<vtkPolyDataMapper> mapper;

  //make a material library so we can put some path tracer material
  //definitions in it
  vtkSmartPointer<vtkOSPRayMaterialLibrary> ml = vtkSmartPointer<vtkOSPRayMaterialLibrary>::New();
  vtkOSPRayRendererNode::SetMaterialLibrary(ml, renderer);

  // read object
  vtkSmartPointer<vtkOBJReader> copperCup =
    vtkSmartPointer<vtkOBJReader>::New();
  copperCup->SetFileName("tmp/white_mug.obj");

  // manually make up a path tracer material for it
  // see also use vtkOSPRayMaterialLibrary for .mtl or .json formats
  // use the ospray :: metal material for purely reflective objects
  ml->AddMaterial("Copper", "Metal");
  // tune the base metal material like so
  double roughness[1] = { 0.0 };
  ml->AddShaderVariable("Copper", "roughness", 1, roughness);
  //for OSP < 1.4 and > 1.4.0 you specific metal "color like so"
  double cuColor[3] = { 0.7843, 0.4588, 0.2 };
  ml->AddShaderVariable("Copper", "reflectance", 3, cuColor);
  //for OSP >= 1.4 you can use the more exact but much less intuitive spectrum
  double spectrum[58*3] = {
    300,     1.347459987,     1.679419071,
    310,     1.321473211,     1.740141215,
    320,     1.301896917,     1.781554261,
    330,     1.278815346,     1.816251273,
    340,     1.257856058,     1.857525737,
    350,     1.229714372,     1.895968733,
    360,     1.205793784,     1.941169403,
    370,     1.183134074,     1.99326522,
    380,     1.16577487 ,     2.046321345,
    390,     1.139929606,     2.090129064,
    400,     1.119339006,     2.14224644,
    410,     1.097661459,     2.193481406,
    420,     1.082884327,     2.251163803,
    430,     1.067185209,     2.306769228,
    440,     1.056310845,     2.361946782,
    450,     1.048210496,     2.413637347,
    460,     1.044058354,     2.464134299,
    470,     1.040826414,     2.50896784,
    480,     1.040383818,     2.549587906,
    490,     1.035622719,     2.577676166,
    500,     1.0292166  ,     2.600958825,
    510,     1.01596237 ,     2.610628188,
    520,     0.995463808,     2.613856957,
    530,     0.957525814,     2.60358516,
    540,     0.896412084,     2.584135179,
    550,     0.79745994 ,     2.56420404,
    560,     0.649913539,     2.566649101,
    570,     0.467667795,     2.633707115,
    580,     0.308052581,     2.774526337,
    590,     0.206477543,     2.953105649,
    600,     0.15342929 ,     3.124794481,
    610,     0.129738592,     3.28082796,
    620,     0.116677068,     3.422223479,
    630,     0.110069919,     3.546563885,
    640,     0.107194012,     3.666809315,
    650,     0.104232496,     3.775693898,
    660,     0.102539467,     3.879628119,
    670,     0.102449402,     3.981770445,
    680,     0.101216009,     4.082308744,
    690,     0.101603953,     4.175083635,
    700,     0.101236908,     4.27062629,
    710,     0.101557633,     4.365353818,
    720,     0.101132194,     4.453675754,
    730,     0.100848965,     4.541494304,
    740,     0.100919789,     4.632837662,
    750,     0.101173963,     4.718605321,
    760,     0.101837799,     4.806908667,
    770,     0.101672055,     4.890330992,
    780,     0.104166566,     4.985764803,
    790,     0.10154611 ,     5.058785587,
    800,     0.105089997,     5.141307607,
    810,     0.105640925,     5.225721003,
    820,     0.1047717  ,     5.314412207,
    830,     0.108065424,     5.399044187,
    840,     0.106329275,     5.471682183,
    850,     0.106803015,     5.558363688,
    860,     0.10806138 ,     5.64355183,
    870,     0.109423947,     5.718126756
  };
  ml->AddShaderVariable("Copper", "ior", 58*3, spectrum);

  //now the normal stuff to put it into the scene
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(7,8,3);
  actor->SetScale(100,100,100);
  prop = actor->GetProperty();
  // with one exception, make the association the material we just defined
  prop->SetMaterialName("Copper");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(copperCup->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkOBJReader> glassCup =
    vtkSmartPointer<vtkOBJReader>::New();
  glassCup->SetFileName("tmp/plastic_mug.obj");
  //for mostly refractive objects, use osp :: Glass
  ml->AddMaterial("defaultGlass", "Glass");
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(0,-15,3);
  actor->SetScale(100,100,100);
  prop = actor->GetProperty();
  prop->SetMaterialName("defaultGlass");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glassCup->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  //no special material here, just using osp::objmaterial which is
  //the most generic (other than then new principled materal)
  vtkSmartPointer<vtkOBJReader> whiteCup =
    vtkSmartPointer<vtkOBJReader>::New();
  whiteCup->SetFileName("tmp/tri_mug.obj");
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(-17,0,3);
  actor->SetScale(100,100,100);
  prop = actor->GetProperty();
  prop->SetColor(1,1,1);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(whiteCup->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkOBJReader> washer_tray =
    vtkSmartPointer<vtkOBJReader>::New();
  washer_tray->SetFileName("tmp/washer_tray.obj");
  actor = vtkSmartPointer<vtkActor>::New();
  prop = actor->GetProperty();
  prop->SetColor(1,1,1);
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(washer_tray->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  //use one metal material for all of the silverware
  ml->AddMaterial("Aluminum", "Metal");

  vtkSmartPointer<vtkOBJReader> fork =
    vtkSmartPointer<vtkOBJReader>::New();
  fork->SetFileName("tmp/fork.obj");
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(20,-10,3);
  actor->SetScale(1,1,1);
  actor->SetOrientation(90,0,0);
  prop = actor->GetProperty();
  prop->SetMaterialName("Aluminum");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(fork->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkOBJReader> knife =
    vtkSmartPointer<vtkOBJReader>::New();
  knife->SetFileName("tmp/knife.obj");
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(16,-10,3);
  actor->SetScale(1,-1,1);
  actor->SetOrientation(90,0,0);
  prop = actor->GetProperty();
  prop->SetMaterialName("Aluminum");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(knife->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  vtkSmartPointer<vtkOBJReader> spoon =
    vtkSmartPointer<vtkOBJReader>::New();
  spoon->SetFileName("tmp/spoon.obj");
  actor = vtkSmartPointer<vtkActor>::New();
  actor->SetPosition(12,-10,3.5);
  actor->SetScale(1,-1,1);
  actor->SetOrientation(90,0,0);
  prop = actor->GetProperty();
  prop->SetMaterialName("Aluminum");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(spoon->GetOutputPort());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  //demonstrate another use of osp::objmaterial, texture mapping
  vtkSmartPointer<vtkPlaneSource> tabletop =
    vtkSmartPointer<vtkPlaneSource>::New();
  tabletop->SetOrigin(-50,-50,-0.15);
  tabletop->SetPoint1(50,-50,-0.15);
  tabletop->SetPoint2(-50,50,-0.15);

  ml->AddMaterial("Wood", "OBJMaterial");
  vtkSmartPointer<vtkJPEGReader> woodreader =
    vtkSmartPointer<vtkJPEGReader>::New();
  vtkSmartPointer<vtkTexture> woodtextr = vtkSmartPointer<vtkTexture>::New();
  fname = "tmp/wood.jpg";
  woodreader->SetFileName(fname.c_str());
  woodreader->Update();
  woodtextr->SetInputConnection(woodreader->GetOutputPort(0));
  ml->AddTexture("Wood", "map_kd", woodtextr);

  actor = vtkSmartPointer<vtkActor>::New();
  prop = actor->GetProperty();
  prop->SetMaterialName("Wood");
  mapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(tabletop->GetOutputPort());
    actor->SetMapper(mapper);
    renderer->AddActor(actor);

  //position camera
  renderer->GetActiveCamera()->SetPosition(15.49,-57.68,29.93);
  renderer->GetActiveCamera()->SetFocalPoint(6.53,-24.33,12.62);
  renderer->GetActiveCamera()->SetViewUp(0,.46,.88);
  renderer->GetActiveCamera()->SetViewAngle(30.0);

  //lighting
  vtkSmartPointer<vtkLight> light = vtkSmartPointer<vtkLight>::New();
  light->SetPosition(-2,0,10);
  light->SetFocalPoint(0,0,0);
  light->PositionalOff();
  // one ospray specific control, radius to get soft shadows
  vtkOSPRayLightNode::SetRadius(2.0, light);
  renderer->AddLight(light);

  //now draw
  renWin->Render();

  //set up progressive rendering
  vtkSmartPointer<vtkProgressiveRenderLooper> looper =
    vtkSmartPointer<vtkProgressiveRenderLooper>::New();
  looper->RenderWindow = renWin;
  vtkCamera *cam = renderer->GetActiveCamera();
  iren->AddObserver(vtkCommand::KeyPressEvent, looper);
  cam->AddObserver(vtkCommand::ModifiedEvent, looper);
  iren->CreateRepeatingTimer(10); //every 10 msec we'll rerender if needed
  vtkOSPRayRendererNode::SetMaxFrames(100, renderer); //up to 100 frames
  iren->AddObserver(vtkCommand::TimerEvent, looper);

  iren->Start();
  return 0;
}
