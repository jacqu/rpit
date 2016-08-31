#include "communicate.h"

/*
 * commutil.c --- support for AVI.mex
 *
 * This module maintains a linked list of AVI file and stream identifiers.  Each 
 * call to AVI('open',...) generates a unique file and stream identifier.  Since 
 * these identifiers are INTERFACE types (COM) they must be kept in memory.  Also, 
 * in case of a premature "clear mex", "quit" or an accidental crash, the identifiers
 * in the list can be closed. 
 *
 */

/* 
 * Copyright 1984-2000 The MathWorks, Inc. 
 * $Revision: 1.1 $  $Date: 2000/01/30 16:14:30 $
*/

static char rcsid[] = "$Id: aviutil.c,v 1.1 2000/01/30 16:14:30 clawton Exp $";

static NodeType OpenFiles;

/* 
 * InitNode 
 *	Initialize the list.
 */
void InitNode(void)
{
	OpenFiles.number = -1;
	OpenFiles.hComm = NULL;
	OpenFiles.next = NULL;
	OpenFiles.previous = NULL;
}

/*
 * addNodetoList
 *	Insert a new node at the begninning of the OpenFiles list.  
 *
 *	Inputs:  pfile        - AVI file identifier
			 psCompressed - Video stream identifier
 *   
 *	Outputs: A unique identification number
 */
int addNodetoList(HANDLE hComm)
{
	NodeType *NewNode = NULL;
    NodeType *ListHead;

	ListHead = &OpenFiles;

	NewNode = (NodeType *) mxMalloc(sizeof(NodeType));
	if (NewNode != NULL)
	{
		mexMakeMemoryPersistent((void *) NewNode);
		NewNode->number = (ListHead->next == NULL)?1:ListHead->next->number+1;
		NewNode->hComm = hComm;
		//NewNode->psCompressed = psCompressed;
		NewNode->next = NULL;
		NewNode->previous = NULL;
	}
	else
		mexErrMsgTxt("Out of memory in AVI MEX-file");

	
	/* Add node to begining of list */
	NewNode ->next = ListHead->next;
	NewNode->previous = ListHead;
	ListHead->next = NewNode;
	if (NewNode->next != NULL)
    {
        NewNode->next->previous = NewNode;
    }

	return(NewNode->number);
}

/* 
 * FindNodeInList
 *	Find a node in the OpenFiles list given a unique identification number.
 *
 *	Inputs:  number - a unique number identifying the node
 *	Outputs: pointer to the node found
 */
NodeType *FindNodeInList(int number)
{

	NodeType *current;
	int found = 0;

	current =  &OpenFiles;
	current = current->next;
	while (current != NULL)
	{
		if(current->number == number)
		{
			found = 1;
			break;
		}
		current = current->next;
	}
	if (found == 0)
		current = NULL;

	return(current);
}

/*
 * deleteNodeFromList
 *	Remove a node from the OpenFiles list given a unique identification number.
 *	
 *	Inputs: number - a unique number identifying the node.
 *  Outputs: none
 */

void deleteNodeFromList(int number)
{
	NodeType *MatchNode;
	NodeType *ListHead;

	ListHead = &OpenFiles;

	MatchNode = FindNodeInList(number);

	if(MatchNode != NULL)
	{
		MatchNode->previous->next = MatchNode->next;
		if (MatchNode->next != NULL)
			MatchNode->next->previous = MatchNode->previous;
		mxFree((void *)MatchNode);
	}
	else
		mexErrMsgTxt("Unable to find node in list");


}	

/* 
 * cleanList
	For each node in the OpenFiles list, close the stream and the file. Also close
 *  the AVIFILE interface. 
 */

void cleanList(void)
{
	NodeType *ListHead;
	NodeType *current;

	ListHead = &OpenFiles;

	while (ListHead->next != NULL)
	{
		current = ListHead->next;

		if (current->hComm)
		{
			//AVIFileClose(current->pfile);クローズハンドルる？
            CloseHandle(current->hComm);
			current->hComm = NULL;
		}

		ListHead->next = current->next;
		mxFree((void *)current);
	}
	     
     //AVIFileExit();
 }
		

void exitMex(void)
{
	NodeType *ListHead;

	ListHead = &OpenFiles;

	if (ListHead ->next != NULL)
	{
		mexWarnMsgTxt("Closing all open AVI files. It is no longer possible to write to any previously open AVI files.");
		cleanList();
	}
	
}
