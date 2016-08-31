#include <windows.h>
#include "mex.h"


typedef struct node
{
	int number;                              // Unique identifier
	HANDLE hComm;                            // File handle
	struct node *next;                       //Pointer to the next     NodeType Struct
	struct node *previous;                   //Pointer to the previous NodeType Struct
} NodeType;

void comopen(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[]);
void comclose(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[]);
void comcloseall(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[]);
void comwrite(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[]);
void comread(int nlhs,
                 mxArray *plhs[],
                 int nrhs,
                 const mxArray *prhs[]);

void cleanList(void);                        //used in exitMex function

void InitNode(void);                         //used in MEX gateway function

int addNodetoList(HANDLE);                  //used in comopen function

NodeType *FindNodeInList(int);              //used in comclose/comwrite function

void deleteNodeFromList(int);              //used in comclose function

void exitMex(void);                         //used in MEX gateway function
