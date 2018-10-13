#include <stdlib.h>
#include <stdio.h>

#include "Python.h"
#include "cnstrmgr.h"
#include "basegrph.h"
#include "capsep.h"
#include "mstarsep.h"
#include "memmod.h"
#include "brnching.h"
#include "glmsep.h"
#include "fcisep.h"
#include "combsep.h"
#include "htoursep.h"

CnstrMgrPointer CMPExistingCuts = NULL;

static PyObject *MyFunction( PyObject *self, PyObject *args ){
  /* int x = PyObject_GetItem(args,0); 
  PyTypeObject* t = o-> ob_type;
  */
  printf("coucou: %d\n", (int) PyTuple_Size(args));
  PyObject* list = PyObject_GetItem(args,PyLong_FromLong(0));
  PyObject* integer = PyObject_GetItem(list,PyLong_FromLong(0));
  long integerC = PyLong_AsLong(integer);
  printf("args[0]= %d\n",integerC);
  Py_RETURN_NONE;
}

static void save_cuts(CnstrMgrPointer CutsCMP){
  if ( ::CMPExistingCuts == NULL ) CMGR_CreateCMgr(&::CMPExistingCuts,100);
  for (int i=0; i<CutsCMP->Size; i++) CMGR_MoveCnstr(CutsCMP,::CMPExistingCuts,i,0);
}

static void init_old_cuts_manager(){ if ( ::CMPExistingCuts == NULL ) CMGR_CreateCMgr(&::CMPExistingCuts,100); }

static PyObject* column_generation( PyObject *self, PyObject *args){
  /* will receive tuple consisting of:
      number of customers 
      Demand vector [(0),1,....n] of length NoOfCustomers+1 so that indexing is 1-based
      capacity
      number of edges
      Edge X, Edge Head, Edge Tail : all 1-based and depot is in the last position
      QMin (for the STI)
  */
  
  int NoOfCustomers = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(0)));
  PyObject* demand_list = PyObject_GetItem(args,PyLong_FromLong(1));
  int CAP = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(2)));
  int NoOfEdges = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(3)));
  PyObject* x_list = PyObject_GetItem(args,PyLong_FromLong(4));
  PyObject* head_list = PyObject_GetItem(args,PyLong_FromLong(5));
  PyObject* tail_list = PyObject_GetItem(args,PyLong_FromLong(6));
  int QMin = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(7)));
  
  double* EdgeX = MemGetDV(NoOfEdges+1);
  int* EdgeHead = MemGetIV(NoOfEdges+1);
  int* EdgeTail = MemGetIV(NoOfEdges+1);
  int* Demand = MemGetIV(NoOfCustomers+2);

  printf("Memory allocated\n");

  for(int i=1; i<NoOfCustomers+1; i++){
    Demand[i] = PyLong_AsLong( PyObject_GetItem( demand_list , PyLong_FromLong(i) ) );
  }
  Demand[NoOfCustomers+1] = 0;

  for(int i=1; i<NoOfEdges+1; i++){
    EdgeX[i] = PyFloat_AsDouble( PyObject_GetItem( x_list , PyLong_FromLong(i) ) ); 
    EdgeHead[i] = PyLong_AsLong( PyObject_GetItem( head_list , PyLong_FromLong(i) ) );
    EdgeTail[i] = PyLong_AsLong( PyObject_GetItem( tail_list , PyLong_FromLong(i) ) );
    if(EdgeHead[i]<=0||EdgeHead[i]>NoOfCustomers+1) printf("Inaccurate head pointer %d\n", EdgeHead[i]);
    if(EdgeTail[i]<=0||EdgeTail[i]>NoOfCustomers+1) printf("Inaccurate pointer %d\n", EdgeTail[i]);
  }

  printf("Python object scanned\n");
 
  char IntegerAndFeasible;
  double EpsForIntegrality = 0.0001;
  double MaxCapViolation;
  int Dim = 31;
  int MaxNoOfCapCuts = 30;
  CnstrMgrPointer CutsCMP;
  CMGR_CreateCMgr(&CutsCMP,Dim);   

  init_old_cuts_manager();

  printf("Objects initialized\n");

  //capacity cuts
  CAPSEP_SeparateCapCuts(NoOfCustomers,Demand,CAP,NoOfEdges,EdgeTail,EdgeHead,EdgeX,::CMPExistingCuts,MaxNoOfCapCuts,EpsForIntegrality,&IntegerAndFeasible,&MaxCapViolation,CutsCMP);
 
  if ( MaxCapViolation < 0.1 ){
    //multistar cuts
    int MaxNoOfMStarCuts = MaxNoOfCapCuts - CutsCMP->Size;
    double MaxMStarViolation;
    MSTARSEP_SeparateMultiStarCuts(NoOfCustomers,Demand,CAP,NoOfEdges,EdgeTail,EdgeHead,EdgeX,::CMPExistingCuts,MaxNoOfMStarCuts,&MaxMStarViolation,CutsCMP);
  }

  printf("Large multistar inequalities\n");
  //Large Multistar inequalities
  int* GLMCutList = MemGetIV(NoOfEdges+1);
  int GLMCutListSize;
  double GLMViolation;
  GLMSEP_SeparateGLM(NoOfCustomers,Demand,CAP,NoOfEdges,EdgeTail,EdgeHead,EdgeX,GLMCutList,&GLMCutListSize,&GLMViolation);

  printf("framed capacity inequalities\n");
  // Framed capacity inequalities
  double MaxFCIViolation;
  int FCIMaxNoOfTreeNodes=100;
  int MaxNoOfFCICuts = MaxNoOfCapCuts - CutsCMP->Size;
  FCISEP_SeparateFCIs(NoOfCustomers, Demand, CAP, NoOfEdges, EdgeTail, EdgeHead, EdgeX, ::CMPExistingCuts, FCIMaxNoOfTreeNodes, MaxNoOfFCICuts, &MaxFCIViolation, CutsCMP);

  printf("strenghtened comb inequalities\n");
  // Stregthened comb inequalities
  double MaxSCIViolation;
  int MaxNoOfSCICuts = MaxNoOfCapCuts - CutsCMP->Size;
  COMBSEP_SeparateCombs(NoOfCustomers, Demand, CAP, QMin, NoOfEdges, EdgeTail, EdgeHead, EdgeX, MaxNoOfSCICuts, &MaxSCIViolation, CutsCMP);

  printf("hypotour inequalities\n");
  // Hypotour inequalities
  double MaxHTIViolation;
  int MaxNoOfHTICuts = MaxNoOfCapCuts - CutsCMP->Size; 
  printf("calling with max %d cuts\n", MaxNoOfHTICuts);
  HTOURSEP_SeparateHTours(NoOfCustomers, Demand, CAP, NoOfEdges, EdgeTail, EdgeHead, EdgeX, ::CMPExistingCuts, MaxNoOfHTICuts, &MaxHTIViolation, CutsCMP);

  //Retrieving the cuts

  printf("Routines called, start retrieving cuts\n");
  PyObject* Cuts = PyList_New(0);

  //REMEMBER : in this cvrp library, lists are 1-based and the depot is coded as number n+1! 

  //Capacity, multistar, framed and strengthened comb inequalities
  for (int i=0; i<CutsCMP->Size; i++) {
    PyObject* Cut = PyList_New(0);
    if (CutsCMP->CPL[i]->CType == CMGR_CT_CAP){
      PyList_Append(Cut,PyUnicode_FromString("cap"));
      for (int j=1; j<=CutsCMP->CPL[i]->IntListSize; j++){
        PyList_Append(Cut,PyLong_FromLong(CutsCMP->CPL[i]->IntList[j]));
      }
    }
    if (CutsCMP->CPL[i]->CType == CMGR_CT_MSTAR){
      PyList_Append(Cut,PyUnicode_FromString("mstar"));
      //Nuclues
      PyObject* Nucleus = PyList_New(0);      
      for (int j=1; j<=CutsCMP->CPL[i]->IntListSize; j++){
        PyList_Append(Nucleus,PyLong_FromLong(CutsCMP->CPL[i]->IntList[j]));
      }
      PyList_Append(Cut,Nucleus);
      //Satellites
      PyObject* Satellites = PyList_New(0);      
      for (int j=1; j<=CutsCMP->CPL[i]->ExtListSize; j++){
        PyList_Append(Satellites,PyLong_FromLong(CutsCMP->CPL[i]->ExtList[j]));
      }
      PyList_Append(Cut,Satellites);
      //Connectors
      PyObject* Connectors = PyList_New(0);      
      for (int j=1; j<=CutsCMP->CPL[i]->CListSize; j++){
        PyList_Append(Connectors,PyLong_FromLong(CutsCMP->CPL[i]->CList[j]));
      }
      PyList_Append(Cut,Connectors);
      //Coefficients of the cut
      PyList_Append(Cut,PyLong_FromLong(CutsCMP->CPL[i]->A));
      PyList_Append(Cut,PyLong_FromLong(CutsCMP->CPL[i]->B));
      PyList_Append(Cut,PyLong_FromLong(CutsCMP->CPL[i]->L));
    }
    if (CutsCMP->CPL[i]->CType == CMGR_CT_FCI){
      PyList_Append(Cut,PyUnicode_FromString("fci"));
      for (int j=1; j<=NoOfCustomers; j++) PyList_Append(Cut,PyLong_FromLong(0));
      int MaxIdx = 0;
      int MinIdx = 0;
      for (int SubsetNr=1; SubsetNr<=CutsCMP->CPL[i]->ExtListSize; SubsetNr++){
        MinIdx = MaxIdx+1;
        MaxIdx = MinIdx + CutsCMP->CPL[i]->ExtList[SubsetNr] - 1;
        for (int j=MinIdx; j<=MaxIdx; j++){
          PyObject_SetItem(Cut,PyLong_FromLong(CutsCMP->CPL[i]->IntList[j]),PyLong_FromLong(SubsetNr));
        }
      }
      PyList_Append(Cut,PyFloat_FromDouble(CutsCMP->CPL[i]->RHS));
    }
    if (CutsCMP->CPL[i]->CType == CMGR_CT_STR_COMB){
      int MinIdx;
      int MaxIdx;
      PyList_Append(Cut,PyUnicode_FromString("sci"));
      for (int j=1; j<=NoOfCustomers+1; j++) PyList_Append(Cut,PyList_New(0)); //ATTENTION : depot can be present in this cut! it's number is n+1 
      int NoOfTeeth = CutsCMP->CPL[i]->Key;
      for (int k=1; k<=CutsCMP->CPL[i]->IntListSize; k++){
        PyList_Append(PyObject_GetItem(Cut,PyLong_FromLong(CutsCMP->CPL[i]->IntList[k])),PyLong_FromLong(0));
      }
      for (int t=1; t<=NoOfTeeth; t++){
        MinIdx = CutsCMP->CPL[i]->ExtList[t];
        if (t == NoOfTeeth) MaxIdx = CutsCMP->CPL[i]->ExtListSize;
        else MaxIdx = CutsCMP->CPL[i]->ExtList[t+1] - 1;
        for (int k=MinIdx; k<=MaxIdx; k++){
          PyList_Append(PyObject_GetItem(Cut,PyLong_FromLong(CutsCMP->CPL[i]->ExtList[k])),PyLong_FromLong(t)); /* Node j is in tooth t */
        }
      }
      PyList_Append(Cut,PyFloat_FromDouble(CutsCMP->CPL[i]->RHS));
    }
    if (CutsCMP->CPL[i]->CType == CMGR_CT_TWOEDGES_HYPOTOUR){
      PyList_Append(Cut,PyUnicode_FromString("hti"));
      PyList_Append(Cut,PyList_New(0)); //head
      PyList_Append(Cut,PyList_New(0)); //tail
      PyList_Append(Cut,PyList_New(0)); //coeff
      for (int j=1; j<=CutsCMP->CPL[i]->IntListSize; j++){
        PyList_Append(PyObject_GetItem(Cut,PyLong_FromLong(1)),PyLong_FromLong(CutsCMP->CPL[i]->IntList[j]));
        PyList_Append(PyObject_GetItem(Cut,PyLong_FromLong(2)),PyLong_FromLong(CutsCMP->CPL[i]->ExtList[j]));
        PyList_Append(PyObject_GetItem(Cut,PyLong_FromLong(3)),PyFloat_FromDouble(CutsCMP->CPL[i]->CoeffList[j]));
      }
      PyList_Append(Cut,PyFloat_FromDouble(CutsCMP->CPL[i]->RHS));
    }
    PyList_Append(Cuts,Cut);
  }
  
  //Generalised large multistar inequalities
  if (GLMCutListSize > 0){
    PyObject* Cut = PyList_New(0);
    PyList_Append(Cut,PyUnicode_FromString("glm"));
    for(int i=1; i<=GLMCutListSize; i++){
      PyList_Append(Cut,PyLong_FromLong(GLMCutList[i]));
    }
    PyList_Append(Cuts,Cut);
  }

  save_cuts(CutsCMP);
  printf("Cuts retrieved, returning results\n");

  return Cuts;
}

static PyObject* branching( PyObject *self, PyObject *args){
  /* will receive tuple consisting of:
      number of customers 
      Demand vector [(0),1,....n] of length NoOfCustomers+1 so that indexing is 1-based
      capacity
      number of edges
      Edge X, Edge Head, Edge Tail : all 1-based and depot is in the last position
      
  */
  
  int NoOfCustomers = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(0)));
  PyObject* demand_list = PyObject_GetItem(args,PyLong_FromLong(1));
  int CAP = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(2)));
  int NoOfEdges = PyLong_AsLong(PyObject_GetItem(args,PyLong_FromLong(3)));
  PyObject* x_list = PyObject_GetItem(args,PyLong_FromLong(4));
  PyObject* head_list = PyObject_GetItem(args,PyLong_FromLong(5));
  PyObject* tail_list = PyObject_GetItem(args,PyLong_FromLong(6));
  
  double* EdgeX = MemGetDV(NoOfEdges+1);
  int* EdgeHead = MemGetIV(NoOfEdges+1);
  int* EdgeTail = MemGetIV(NoOfEdges+1);
  int* Demand = MemGetIV(NoOfCustomers+1);

  for(int i=0; i<NoOfCustomers+1; i++){
    Demand[i] = PyLong_AsLong( PyObject_GetItem( demand_list , PyLong_FromLong(i) ) );
  }

  for(int i=0; i<NoOfEdges+1; i++){
    EdgeX[i] = PyFloat_AsDouble( PyObject_GetItem( x_list , PyLong_FromLong(i) ) ); 
    EdgeHead[i] = PyLong_AsLong( PyObject_GetItem( head_list , PyLong_FromLong(i) ) );
    EdgeTail[i] = PyLong_AsLong( PyObject_GetItem( tail_list , PyLong_FromLong(i) ) );
  }

  int MaxNoOfSets,SetNr;
  double BoundaryTarget;
  CnstrMgrPointer SetsCMP;
  BoundaryTarget = 2.7;
  MaxNoOfSets = NoOfCustomers;
  CMGR_CreateCMgr(&SetsCMP,MaxNoOfSets);
  BRNCHING_GetCandidateSets(NoOfCustomers,Demand,CAP,NoOfEdges,EdgeTail,EdgeHead,EdgeX,::CMPExistingCuts,BoundaryTarget,MaxNoOfSets,SetsCMP);

  PyObject* Sets = PyList_New(0);
  for (int i=0; i<SetsCMP->Size; i++){
    PyObject* Set = PyList_New(0);
    for (int j=1; j<=SetsCMP->CPL[i]->IntListSize; j++){
      PyList_Append(Set,PyLong_FromLong(SetsCMP->CPL[i]->IntList[j]));
    }
    PyList_Append(Sets,Set);
  }
CMGR_FreeMemCMgr(&SetsCMP);
return Sets;
}

static PyMethodDef module_methods[] = {
   { "MyFunction", (PyCFunction)MyFunction, METH_VARARGS, "test function for using C/python API" },
   { "column_generation", (PyCFunction)column_generation, METH_VARARGS, "returns the set of nodes that define a capacity cut" },
   { "branching", (PyCFunction)branching, METH_VARARGS, "returns the set of nodes that define a constraint branching" },
   { NULL, NULL, 0, NULL }
};

static struct PyModuleDef moduledef = {
    PyModuleDef_HEAD_INIT,
    "cvrpsep", /* m_name */
    "module to find cutting planes",      /* m_doc */
    -1,                  /* m_size */
    module_methods,    /* m_methods */
    NULL,                /* m_reload */
    NULL,                /* m_traverse */
    NULL,                /* m_clear */
    NULL,                /* m_free */
  };

static PyObject *
moduleinit(void)
{
    PyObject *m;

    m = PyModule_Create(&moduledef);

    if (m == NULL)
        return NULL;

  return m;
}

PyMODINIT_FUNC
    PyInit_cvrpsep(void)
    {
        return moduleinit();
    }
