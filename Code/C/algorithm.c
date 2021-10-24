#include <stdio.h> 
#include <limits.h> 
#include <string.h> 
#include <math.h> 
#ifdef _WIN32
#include <Windows.h>
#include <unistd.h>
#else
#include <unistd.h>
#endif

#define DISTANCETOPARKFROMQR 10
#define ROTATION_CALC 2
#define CALIB1 1
#define CALIB2 1

#define MAXV 220 
#define PI 3.14159265359
#define BIGDIST 30000

#define BLUE 10
#define GREEN 11
#define YELLOW 12
#define RED 13
#define ORANGE 14
#define UNKNOWN 15

#define FRONT 1
#define BACK 2
#define RIGHT 3
#define LEFT 4
#define CALIBRATE 5
#define TAKEANDDROP 6
#define TAKEANDHOLD 7
#define PLACECUBE 8
#define COLORCHECK 9
#define PARK 10
#define STOP 0
#define NOCALIB 15

#define PAINNER 345
#define RADIUS 69

#define CELL 115
#define MAX(a,b) ((a) > (b) ? a : b)
#define MIN(a,b) ((a) < (b) ? a : b)

#define AECLIPSE 72 
#define BECLIPSE 67 

#define AA 18.5
#define BB 26.5
#define CC 45
#define DD 63.5
#define EE 71.5

	
int calcCalibrationDistance() {
	return 157;
}	

double angles[7][7] =  {{-1,-1,180-EE,90,EE,-1,-1},
					  {-1,180-CC,180-DD,-1,DD,CC,-1},
					  {180-AA,180-BB,-1,-1,-1,BB,AA},
					  {180,-1,-1,-1,-1,-1,0},
					  {180+AA,180+BB,-1,-1,-1,360-BB,360-AA},
					  {-1,180+CC,180+DD,-1,360-DD,360-CC,-1},
					  {-1,-1,180+EE,270,360-EE,-1,-1}
            
};

typedef struct {
	int x,y;
} point_t;

typedef struct {
   int boxColor; 
   int cubeColor;
   point_t upperLeft;
   point_t lowerRight;
   point_t park;
   int parkDir;
} box_t;

typedef struct  {
   point_t A,B;
   point_t pathStart;
   point_t entryPoint;
   double dir;
} parkingArea_t;

typedef struct  {
   point_t center;
   double dir;
} robot_t;

typedef struct  {
   point_t p;
   double dir;
} robotPath_t;

robot_t robot;
box_t box[5];
parkingArea_t parkArea;
int pista[21][21]={0};
int pistaTurns[21][21]={0};

point_t map[MAXV];

char robotMoves[800];
robotPath_t robotPath[40];
int robotPathCounter;
 
FILE *r1;
FILE *r2;
FILE *c1;
FILE *cf;
FILE *qr;
FILE *javaFile;

double toRadians(double angleInDegrees)
{
    return (PI / 180) * angleInDegrees;
}

int calcDistanceOnRotationEllipse() {
	if (ROTATION_CALC == 2) return RADIUS;
	if (ROTATION_CALC == 0) 
	     return round (sqrt( AECLIPSE*AECLIPSE*pow(cos(toRadians(robot.dir)),2) + BECLIPSE*BECLIPSE*pow(sin(toRadians(robot.dir)),2)));
	if (ROTATION_CALC == 1) return BECLIPSE;
}

double calcDir(point_t p1, point_t p2) {
	double result;
  	int dx = p2.x-p1.x;
  	int dy = p2.y-p1.y;
  	result = atan2(dy,dx) * 180 / PI;
  	if (result<0) 
    	result = -1*result;
  	else if (result>0) 
    	result = 360-result;   
    return result;	
}

int calcDistance(point_t p1, point_t p2) {
    return round(sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2))); 
}

int minDistance(int dist[], int V, int sptSet[]) 
{
	int min = INT_MAX, min_index; 
	int v;
	
	for (v = 0; v < V; v++) 
		if (sptSet[v] == 0 && dist[v] <= min) 
			min = dist[v], min_index = v;
	return min_index; 
} 

printPoint(point_t p, char *label) {
	printf("%s(%d,%d)",label,p.x,p.y);
}

void printPath(int parent[], int j) 
{
	if (parent[j] == - 1) {
		printf("%c,%c -> ",map[j].x+'A', map[j].y+'A');
		robotPathCounter = 1;
		robotPath[robotPathCounter-1].p.x = map[j].x * CELL;
		robotPath[robotPathCounter-1].p.y = map[j].y * CELL;
		return; 
	}
	printPath(parent, parent[j]); 
	printf("%c,%c -> ",map[j].x+'A', map[j].y+'A');
	robotPathCounter++;
	robotPath[robotPathCounter-1].p.x = map[j].x * CELL;
	robotPath[robotPathCounter-1].p.y = map[j].y * CELL;
}

int dijkstra(int V, unsigned short int graph[MAXV][MAXV], int src, int target) 
{ 
	int dist[V]; 
	int sptSet[V]; 
	int parent[V]; 
	int i, count,v;
	
	for (i = 0; i < V; i++) 
	{ 
		parent[i] = -1; 
		dist[i] = INT_MAX; 
		sptSet[i] = 0; 
	} 

	dist[src] = 0; 

	for (count = 0; count < V - 1; count++) 
	{
		int u = minDistance(dist, V, sptSet);
		sptSet[u] = 1;
		for (v = 0; v < V; v++)
			if (!sptSet[v] && graph[u][v] && dist[u] + graph[u][v] < dist[v]){ 
				parent[v] = u; 
				dist[v] = dist[u] + graph[u][v]; 
			} 
		if (u==target) break;	
	} 
	printf("\nDijkstra %d -> %d \t\t %d\t\t%d ", src, target, dist[target], src); 
	robotPathCounter = 0;
	printPath(parent, target); 
	for (i=0;i<robotPathCounter-1;i++) {
		robotPath[i].dir = calcDir(robotPath[i].p, robotPath[i+1].p);
	}
	return dist[target]; 
}

void parkAreaMasking(double parkDir) {
	int k,m,size,i,j;
    printf("%f\n", parkDir);
	if (parkDir == 0) {
		k = parkArea.A.x-1; m = parkArea.A.y-1; size = 6;
		parkArea.pathStart.x = parkArea.A.x-3;
		parkArea.pathStart.y = parkArea.A.y+1;
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] = 0;
	} else if (parkDir == AA) {
		k = parkArea.A.x-1; m = parkArea.A.y-2; size = 7;
		parkArea.pathStart.x = parkArea.A.x-3; 
		parkArea.pathStart.y = parkArea.A.y+2;
		int paMask[7][7] = {	{1, 1, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 1, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == BB) {
		k = parkArea.A.x-1; m = parkArea.A.y-3; size = 8;
		parkArea.pathStart.x = parkArea.A.x-2;
		parkArea.pathStart.y = parkArea.A.y+2;
		int paMask[8][8] = {	{1, 1, 1, 0, 0, 1, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 1, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 							
								{1, 0, 0, 0, 0, 1, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == CC) {
		k = parkArea.A.x-2; m = parkArea.A.y-4; size = 9;
		parkArea.pathStart.x = parkArea.A.x-1;
		parkArea.pathStart.y = parkArea.A.y+3;
		int paMask[9][9] = {	{1, 1, 1, 1, 0, 0, 1, 1, 1}, 
								{1, 1, 1, 0, 0, 0, 0, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 1, 0, 0, 0, 0, 0, 0, 1}, 							
								{1, 1, 1, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 1, 1, 0, 0, 1, 1, 1} 														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == DD) {
		k = parkArea.A.x-1; m = parkArea.A.y-4; size = 8;
		parkArea.pathStart.x = parkArea.A.x;
		parkArea.pathStart.y = parkArea.A.y+3;
		int paMask[8][8] = {	{1, 0, 0, 0, 0, 1, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 1, 0, 0, 1, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == EE) {
		k = parkArea.B.x-2; m = parkArea.B.y-1; size = 7;
		parkArea.pathStart.x = parkArea.A.x+1;
		parkArea.pathStart.y = parkArea.A.y+3;
		int paMask[7][7] = {	{1, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 1, 0, 0, 0, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == 90) {
		k = parkArea.B.x-1; m = parkArea.B.y-1; size = 6;
		parkArea.pathStart.x = parkArea.A.x+1;
		parkArea.pathStart.y = parkArea.A.y+3;
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] = 0;
	} else if (parkDir == AA+90) {
		k = parkArea.B.x-1; m = parkArea.B.y-2; size = 7;
		parkArea.pathStart.x = parkArea.A.x+2;
		parkArea.pathStart.y = parkArea.A.y+2;
		int paMask[7][7] = {	{1, 1, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 1, 1, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == BB+90) {
		k = parkArea.A.x-3; m = parkArea.A.y-6; size = 8;
		parkArea.pathStart.x = parkArea.A.x+3;
		parkArea.pathStart.y = parkArea.A.y+2;
		int paMask[8][8] = {	{1, 1, 1, 1, 0, 0, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 							
								{1, 1, 0, 0, 0, 1, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == CC+90) {
		k = parkArea.A.x-4; m = parkArea.A.y-6; size = 9;
		parkArea.pathStart.x = parkArea.A.x+3;
		parkArea.pathStart.y = parkArea.A.y+1;
		int paMask[9][9] = {	{1, 1, 1, 0, 0, 0, 1, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 1, 0, 0, 0, 1, 1, 1}, 							
								{1, 1, 1, 1, 1, 1, 1, 1, 1} 														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == DD+90) {
		k = parkArea.A.x-4; m = parkArea.A.y-6; size = 8;
		parkArea.pathStart.x = parkArea.A.x+3;
		parkArea.pathStart.y = parkArea.A.y;
		int paMask[8][8] = {	{1, 1, 0, 0, 1, 1, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 0, 0, 0, 0, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == EE+90) {
		k = parkArea.B.x-1; m = parkArea.B.y-4; size = 7;
		parkArea.pathStart.x = parkArea.A.x+3;
		parkArea.pathStart.y = parkArea.A.y-1;
		int paMask[7][7] = {	{1, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == 180) {
		k = parkArea.B.x-1; m = parkArea.B.y-4; size = 6;
		parkArea.pathStart.x = parkArea.A.x+3;
		parkArea.pathStart.y = parkArea.A.y-1;
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] = 0;
	} else if (parkDir == AA+180) {
		k = parkArea.B.x-2; m = parkArea.B.y-5; size = 7;
		parkArea.pathStart.x = parkArea.A.x+2;
		parkArea.pathStart.y = parkArea.A.y-2;
		int paMask[7][7] = {	{1, 1, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 1, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == BB+180) {
		k = parkArea.A.x-6; m = parkArea.A.y-4; size = 7;
		parkArea.pathStart.x = parkArea.A.x+2;
		parkArea.pathStart.y = parkArea.A.y-2;

		int paMask[8][8] = {	{1, 1, 1, 0, 0, 0, 0, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 1, 0, 0, 0, 0, 0, 1}, 							
								{1, 1, 1, 0, 0, 1, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == CC+180) {
		k = parkArea.A.x-6; m = parkArea.A.y-4; size = 9;
		parkArea.pathStart.x = parkArea.A.x+1;
		parkArea.pathStart.y = parkArea.A.y-3;
		int paMask[9][9] = {	{1, 1, 1, 0, 0, 1, 1, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 1, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 0, 0, 0, 0, 1, 1, 1}, 							
								{1, 1, 1, 0, 0, 1, 1, 1, 1} 														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == DD+180) {
		k = parkArea.A.x-6; m = parkArea.A.y-4; size = 8;
		parkArea.pathStart.x = parkArea.A.x;
		parkArea.pathStart.y = parkArea.A.y-3;
		int paMask[8][8] = {	{1, 1, 1, 0, 0, 1, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 							
								{1, 1, 1, 0, 0, 0, 0, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == EE+180) {
		k= parkArea.A.x-5; m = parkArea.A.y-2; size = 7;
		parkArea.pathStart.x = parkArea.A.x-1;
		parkArea.pathStart.y = parkArea.A.y-3;
		int paMask[7][7] = {	{1, 0, 0, 0, 1, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == 270) {
		k = parkArea.A.x-4; m = parkArea.A.y-1; size = 6;
		parkArea.pathStart.x = parkArea.A.x-1;
		parkArea.pathStart.y = parkArea.A.y-3;
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] = 0;

	} else if (parkDir == AA+270) {
		k = parkArea.A.x-4; m = parkArea.A.y-1; size = 7;
		parkArea.pathStart.x = parkArea.A.x-2;
		parkArea.pathStart.y = parkArea.A.y-2;
		int paMask[7][7] = {	{1, 1, 1, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 1, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == BB+270) {
		k = parkArea.A.x-4; m = parkArea.A.y-1; size = 8;
		parkArea.pathStart.x = parkArea.A.x-3;
		parkArea.pathStart.y = parkArea.A.y-1;
		int paMask[8][8] = {	{1, 1, 1, 0, 0, 0, 1, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 0, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 0, 0, 1, 1, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == CC+270) {
		k = parkArea.A.x-4; m = parkArea.A.y-2; size = 9;
		parkArea.pathStart.x = parkArea.A.x-3;
		parkArea.pathStart.y = parkArea.A.y-1;
		int paMask[9][9] = {	{1, 1, 1, 1, 1, 1, 1, 1, 1}, 
								{1, 1, 1, 0, 0, 0, 1, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 1}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0, 1}, 							
								{1, 1, 0, 0, 0, 0, 0, 1, 1}, 							
								{1, 1, 1, 0, 0, 0, 1, 1, 1} 														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == DD+270) {
		k = parkArea.A.x-3; m = parkArea.A.y-1; size = 8;
		parkArea.pathStart.x = parkArea.A.x-3;
		parkArea.pathStart.y = parkArea.A.y;
		int paMask[8][8] = {	{1, 1, 0, 0, 0, 0, 1, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 0, 1}, 							
								{1, 1, 1, 1, 0, 0, 1, 1}  														
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	} else if (parkDir == EE+270) {
		k = parkArea.A.x-2; m = parkArea.A.y-1; size = 7;
		parkArea.pathStart.x = parkArea.A.x-3;
		parkArea.pathStart.y = parkArea.A.y+1;
		int paMask[7][7] = {	{1, 0, 0, 0, 0, 1, 1}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{1, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 0}, 
								{0, 0, 0, 0, 0, 0, 1}, 
								{1, 1, 0, 0, 0, 0, 1} 							
						}; 
		for (i=0;i<size;i++) for (j=0;j<size;j++) if (k+i>=2 && k+i<19 && m+j>=2 && m+j<19) 
			pista[k+i][m+j] *= paMask[j][i];
	}
	if (parkArea.pathStart.x<=1) parkArea.pathStart.x++;
	if (parkArea.pathStart.y<=1) parkArea.pathStart.y++;
	if (parkArea.pathStart.x>=19) parkArea.pathStart.x--;
	if (parkArea.pathStart.y>=19) parkArea.pathStart.y--;
}

point_t calculateInterceptionPoint(point_t A, point_t B, point_t C, point_t D) {
    double a1 = B.y - A.y; 
    double b1 = A.x - B.x; 
    double c1 = a1*(A.x) + b1*(A.y); 
  
    double a2 = D.y - C.y; 
    double b2 = C.x - D.x; 
    double c2 = a2*(C.x)+ b2*(C.y); 
  
    double determinant = a1*b2 - a2*b1; 
    double x,y;
    if (determinant == 0) { 
        x=-1;
        y=-1;
    } else { 
        x = (b2*c1 - b1*c2)/determinant; 
        y = (a1*c2 - a2*c1)/determinant; 
    } 
    point_t interceptionPoint;
	interceptionPoint.x = round(x);
	interceptionPoint.y = round(y);

    return interceptionPoint;
}

void breakMovements(int dir,int mm){
	sprintf(robotMoves,"%s%d,%d,",robotMoves,dir,mm);
}

void moveFront(int mm) {
	fprintf(javaFile,"gc.setColor(Color.RED);\ngc.drawString(\"%d\",v(%d),v(%d));\ngc.drawLine(v(%d),v(%d)", mm,robot.center.x+10, robot.center.y+10, robot.center.x, robot.center.y);
    robot.center.x = round(robot.center.x + mm * cos(toRadians(robot.dir)));
    robot.center.y = round(robot.center.y - mm * sin(toRadians(robot.dir)));
    fprintf(javaFile,",v(%d),v(%d));\n", robot.center.x, robot.center.y);
    
	breakMovements(FRONT,mm);
}


void moveBack(int mm) {
    double oppositeRobotDir = robot.dir+180;
	if (oppositeRobotDir>=360) oppositeRobotDir-=360;

    fprintf(javaFile,"gc.setColor(Color.BLUE);\ngc.drawString(\"%d\",v(%d),v(%d));\ngc.drawLine(v(%d),v(%d)", mm,robot.center.x+10, robot.center.y+10, robot.center.x, robot.center.y);
    robot.center.x = round(robot.center.x + mm * cos(toRadians(oppositeRobotDir)));
    robot.center.y = round(robot.center.y - mm * sin(toRadians(oppositeRobotDir)));
    fprintf(javaFile,",v(%d),v(%d));\n", robot.center.x, robot.center.y);

	breakMovements(BACK,mm);
}

void moveBackFromBox() {
	int mm = calcCalibrationDistance();
	breakMovements(BACK,mm);
}

void turnTo(point_t turnPoint, double newDir) {
	if (fabs(newDir - robot.dir)<0.1) return;
	
	int tempx=robot.center.x;
	int tempy=robot.center.y;
	int rotationCycleRadian = RADIUS;
	int turnDir;
	
	if (ROTATION_CALC==2) {
		robot.center.x = round(turnPoint.x + rotationCycleRadian * cos(toRadians(newDir)));
		robot.center.y = round(turnPoint.y - rotationCycleRadian * sin(toRadians(newDir)));
	} else if (ROTATION_CALC==1) {
		robot.center.x = round(turnPoint.x + AECLIPSE * cos(toRadians(newDir))-AECLIPSE * cos(toRadians(robot.dir)));
		robot.center.y = round(turnPoint.y - BECLIPSE * sin(toRadians(newDir))+BECLIPSE * sin(toRadians(robot.dir)));			
	} else {
		robot.center.x = round(turnPoint.x + AECLIPSE * cos(toRadians(newDir)));
		robot.center.y = round(turnPoint.y - BECLIPSE * sin(toRadians(newDir)));
	}
	
	fprintf(javaFile,"gc.setColor(new Color(102,51,0)); //BROWN\ngc.fillRect(v(%d-3),v(%d-3),6,6);\n", tempx,tempy);
	fprintf(javaFile,"gc.setColor(new Color(255,204,51)); //GOLD\ngc.fillRect(v(%d-3),v(%d-3),6,6);\n",  robot.center.x,robot.center.y);

	double angle;
	double tempAngle = robot.dir - newDir; 
	
	if (tempAngle>180) {
		tempAngle = -360+tempAngle;
	} else if (tempAngle<-180){
		tempAngle = 360+tempAngle;
	}
	fprintf(javaFile,"\n// ANGLES: %f %f %f\n", robot.dir, newDir, tempAngle);

    if (tempAngle<0) {
    	angle = round(-2*tempAngle)/2.0;
    	turnDir = LEFT;
	}
    else {
    	angle = round(2*tempAngle)/2.0;
    	turnDir = RIGHT;    	
	}
	int x,y;
	x=round(turnPoint.x/CELL);
	y=round(turnPoint.y/CELL);
	int calcTurnDir,maneuver=0;
	if (pistaTurns[x][y]>0) {
		printf("CLOSED CORNER: x=%d y=%d robotDir=%f newDir=%f ",x,y,robot.dir,newDir);
	    calcTurnDir = findValidTurn(x, y, robot.dir, newDir);
	    
		if (calcTurnDir!=RIGHT && calcTurnDir!=LEFT) {
			printf("\n\n******************\nBIG TURNING PROBLEM!!!! turnDir = %d\n\n",calcTurnDir);
			maneuver=1;
		} else {
			if (turnDir!=calcTurnDir) {
				turnDir = calcTurnDir;
				angle = 360-angle;
			}
		}
	}
	if (maneuver==1) {
		sprintf(robotMoves,"%s2,30,%d,180,2,30,",robotMoves,turnDir);
		if (turnDir==LEFT && angle!=180) {
			sprintf(robotMoves,"%s%d,%.1f,",robotMoves,RIGHT,180-angle);
		} else if (turnDir==RIGHT  && angle!=180) {
			sprintf(robotMoves,"%s%d,%.1f,",robotMoves,LEFT,180-angle);
		}
		fprintf(javaFile,"gc.setColor(Color.GREEN);\ngc.drawString(\"maneuver%.0f\",v(%d),v(%d));\ngc.drawArc(v(%d),v(%d),v(%d),v(%d),%.0f,%.0f);\n",round(tempAngle),
	tempx,tempy,turnPoint.x - RADIUS, turnPoint.y - RADIUS, 2*RADIUS, 2*RADIUS, round(robot.dir), round((-1)*tempAngle));
	} else {
		sprintf(robotMoves,"%s%d,%.1f,",robotMoves,turnDir,angle);
		fprintf(javaFile,"gc.setColor(Color.GREEN);\ngc.drawString(\"%.0f\",v(%d),v(%d));\ngc.drawArc(v(%d),v(%d),v(%d),v(%d),%.0f,%.0f);\n",round(tempAngle),
	tempx,tempy,turnPoint.x - RADIUS, turnPoint.y - RADIUS, 2*RADIUS, 2*RADIUS, round(robot.dir), round((-1)*tempAngle));
	}
	
	robot.dir = newDir;
}


void moveToNextBox() {
	int i, distance;
	turnTo(robotPath[0].p,robotPath[0].dir);
 	for (i=1;i<robotPathCounter;i++) {
 		if (fabs(robotPath[i].dir-robot.dir)<0.2 && i<robotPathCounter-1) continue;
		distance = calcDistance(robot.center, robotPath[i].p);
	    distance = distance + calcDistanceOnRotationEllipse();
	    moveFront(distance);
	 	turnTo(robotPath[i].p,robotPath[i].dir); 		
	}
}

void javaInsertPoint(point_t p, char *color, int size) {
	fprintf(javaFile,"gc.setColor(Color.%s);\ngc.drawOval(v(%d),v(%d),v(%d),v(%d));\n" ,color,p.x-size,p.y-size,2*size+1,2*size+1);
}

int theOtherCube(int a, int b) {
	if (a!=0 && b!=0) return 0;
	if (a!=1 && b!=1) return 1;
	if (a!=2 && b!=2) return 2;
}

int protos, deyteros, tritos;

char * cm(char color) {
	switch (color) {
		case BLUE:
			return "BLUE";
		case RED:
			return "RED";
		case ORANGE:
			return "ORANGE";
		case YELLOW:
			return "YELLOW";
		case GREEN:
			return "GREEN";
		default:
			return "GRAY";
	}
}

void printBoxesInOrder() {
	printf("ORDER: %d %d %d\n",protos,deyteros,tritos);
	printf("%s (%s) -> %s (%s) -> %s (%s)\n",cm(box[protos].boxColor),cm(box[protos].cubeColor),cm(box[deyteros].boxColor),cm(box[deyteros].cubeColor),
		cm(box[tritos].boxColor),cm(box[tritos].cubeColor));
	int i;
    for (i=0;i<5;i++) {
		fprintf(javaFile,"gc.setColor(Color.%s);\n",cm(box[i].boxColor));
	    fprintf(javaFile,"Rectangle rr%d = new Rectangle(v(%d*CELL),v(%d*CELL),v(2*CELL),v(2*CELL));\n",i,box[i].upperLeft.x,box[i].upperLeft.y); 
	    fprintf(javaFile,"gc.fill(rr%d);\n",i);
		fprintf(javaFile,"gc.setColor(Color.%s);\n",cm(box[i].cubeColor));
	    fprintf(javaFile,"Rectangle ss%d = new Rectangle(v(%d*CELL+95),v(%d*CELL+95),v(40),v(40));\n",i,box[i].upperLeft.x,box[i].upperLeft.y); 
	    fprintf(javaFile,"gc.fill(ss%d);\n",i);
	}
    
}

int findPosOfMin(int a[], int size) {
	int i,pos=0,min=a[0];
	for (i=1;i<size;i++) {
		if (a[i]<min) {
			min = a[i];
			pos = i;
		}
	}
	return pos;
}

void turnsEncoding(int x, int y, int turnCode) {
	if (x>=19 || y>=19 || x<2 || y<2) return;
	if (pista[x,y] == 0) return;
	
	if (pistaTurns[x][y]==0) {
	     pistaTurns[x][y] = turnCode;
	     printf("[%d][%d]=%d, ",x,y,turnCode);
	}
	else {
	 	 pistaTurns[x][y] = pistaTurns[x][y] + turnCode*100;
	 	 printf("[%d][%d]=%d, ",x,y,pistaTurns[x][y]);
	}
}

int findValidTurn(int x, int y, double robotDir, double newDir) {
	int temp = pistaTurns[x][y];
	int from, to, turnDir=1, turnCode ;
	int rtl,rfl,rnl,rtr,rfr,rnr;
	printf("\nfindValidTurn [%d][%d]=%d: ",x,y,pistaTurns[x][y]);
	while (temp>0) {
		turnCode = temp%100;
		temp = temp/100;
		switch (turnCode)
		{
			case 11: from=330; to=225; break;
			case 12: from=330; to=210; break;
			case 13: from=315; to=210; break;
			case 21: from=240; to=135; break;
			case 22: from=240; to=120; break;
			case 23: from=225; to=120; break;
			case 31: from=150; to=45; break;
			case 32: from=150; to=30; break;
			case 33: from=135; to=30; break;
			case 41: from=60; to=315; break;
			case 42: from=60; to=300; break;
			case 43: from=45; to=300; break;
		}
		rtl = calcLeftArc((int)round(robotDir),to);
		rfl = calcLeftArc((int)round(robotDir),from);
		rnl = calcLeftArc((int)round(robotDir),(int)round(newDir));
		
		printf("from:%d,to:%d,rtl:%d,rfl:%d,rnl:%d",from,to,rtl,rfl,rnl);
		if (rfl==0) {
			if (rtl>=rnl-1)
				turnDir *= 4;
			else	
				turnDir *= 5;
		} else if (rfl<rtl) {
					if (rtl==0 && rnl>=rfl-1) 
						turnDir *= 3;
					else
						turnDir *= 5;
		} else {
			if (rnl>=rfl-1) 
				turnDir *= 3;
			else if (rnl<=rtl+1)
				turnDir *= 4;
			else
				turnDir *= 5;
		}	
	}
	printf("->%d",turnDir);
	if (turnDir==4 || turnDir==16 || turnDir==64) return 4;
	if (turnDir==3 || turnDir==9 || turnDir==27) return 3;
	return turnDir;
}

int calcLeftArc(int gonia1, int gonia2) {
	int t;
    
	if (gonia2>=gonia1)
		t=gonia2-gonia1;
	else
		t=360-(gonia1-gonia2);
	if (t==1 || t==359) t=0;
    printf("{%d %d %d}",gonia1,gonia2,t);
	return t;	
}

int checkTurnCode(int x, int y, int turnCode2Check) {
	int turnCode,temp = pistaTurns[x][y];
	while (temp>0) {
		turnCode = temp%100;
		temp = temp/100;
		if (turnCode == turnCode2Check) {
			return 1;
		}
	}
	return 0; 	
}

int main( int argc, char *argv[] )
{ 
    #ifdef _WIN32
	   	char route1[] = "route1.csv";
	   	char route2[] = "route2.csv";   	
	   	char cubes3Colors[] = "cubes3Colors.txt";
	   	char cubes4Colors[] = "cubes4Colors.txt";
	   	char cubes3Flag[] = "cubes3Flag.txt";

	   	char qrFile[] = "qr.txt";
	   	char jFile[] = "DrawCode.java";
    #else
	   	char route1[] = "/home/lvuser/route1.csv";
	   	char route2[] = "/home/lvuser/route2.csv";   	
	   	char cubes3Colors[] = "/home/lvuser/cubes3Colors.txt";
	   	char cubes4Colors[] = "/home/lvuser/cubes4Colors.txt";
	   	char cubes3Flag[] = "/home/lvuser/cubes3Flag.txt";
	   	
	   	char qrFile[] = "/home/lvuser/qr.txt";
	   	char jFile[] = "/home/lvuser/DrawCode.java";
    #endif


	javaFile = fopen(jFile, "w");
	
	int ret;
	char QRcode[60];
	int c;

	printf("WAITING FOR QR....\n");
	int foundQr = 0;
	do {
		sleep(0.5);
		if( foundQr == 0 && access( qrFile, F_OK ) != -1 ) 
		    foundQr = 1;
	} while (foundQr==0);

	qr = fopen(qrFile, "r");
	int count = 0;
	if (qr) {
	    while ((c = getc(qr)) != EOF) {
	        QRcode[count] = c;
			count++;
	        if (count==54)
	            break;
		}
	    QRcode[count] = '\0';
    	fclose(qr);
    	if (count!=54) {
    		printf("PROBLEM WITH THE TEXT IN QR FILE\n");
    		return 0;
		} else {
			printf("%s\n",QRcode);		
		}
	}
	char letters[25];
	int i,j;

    for (i=2;i<19;i++)
        for (j=2;j<19;j++)
            pista[i][j]=1;

    int len = strlen(QRcode);
    int k=0;
    for (i=0; i<len; i++) {
      if (QRcode[i]>='A' && QRcode[i]<='Z') {
      	letters[k]=QRcode[i];
      	k++;	
	  }
    }
    letters[k]='\0';

	int xp1 = (letters[0] - 'A');
	int yp1 = (letters[1] - 'A');
	int xp2 = (letters[2] - 'A');
	int yp2 = (letters[3] - 'A');
	
	parkArea.dir = angles[3+yp2-yp1][3+xp2-xp1];
	parkArea.A.x = xp1;
	parkArea.A.y = yp1;
	parkArea.B.x = xp2;
	parkArea.B.y = yp2;
	
	int A_RC = PAINNER/2;
	int A_RC_angle = 90;
	    
	parkArea.entryPoint.x = round(xp1*CELL + A_RC * cos(toRadians(parkArea.dir - A_RC_angle)));
	parkArea.entryPoint.y = round(yp1*CELL - A_RC * sin(toRadians(parkArea.dir - A_RC_angle)));
	
	
	parkAreaMasking(parkArea.dir);
	
	int boxMinDistX, boxMinDistY, tempx1, tempy1, tempx2, tempy2, x1,y1,x2,y2,p;
	int boxParkPointDist = 3;
	for (p=0;p<5;p++)
	{
		tempx1 = (letters[4*p+4] - 'A');
		tempy1 = (letters[4*p+5] - 'A');
		tempx2 = (letters[4*p+6] - 'A');
		tempy2 = (letters[4*p+7] - 'A');

		x1 = MIN(tempx1,tempx2);
		y1 = MIN(tempy1,tempy2);
		x2 = MAX(tempx1,tempx2);
		y2 = MAX(tempy1,tempy2);
		
		box[p].upperLeft.x = x1; 
		box[p].upperLeft.y = y1;
		box[p].lowerRight.x = x2;
		box[p].lowerRight.y = y2;
		box[p].boxColor = UNKNOWN;
		box[p].cubeColor = UNKNOWN;
		
		if (x1>=10) {
		   boxMinDistX = 20-x2;
		   box[p].park.x = x1 - boxParkPointDist;  
		   box[p].park.y = y1 + 1;
		   box[p].parkDir = 0;
		}
		else {
		   boxMinDistX = x1;
		   box[p].park.x = x2 + boxParkPointDist;
		   box[p].park.y = y1 + 1;
		   box[p].parkDir = 180;
		}
		if (y1>=10) {
		   boxMinDistY = 20-y2;
		   if (boxMinDistY < boxMinDistX) {
			   box[p].park.x = x1 + 1;
			   box[p].park.y = y1 - boxParkPointDist;
			   box[p].parkDir = 270;
		   }
		}
		else {
		   boxMinDistY = y1;
		   if (boxMinDistY < boxMinDistX) {
			   box[p].park.x = x1 + 1;
			   box[p].park.y = y2 + boxParkPointDist;
			   box[p].parkDir = 90;
		   }
		}
		pista[box[p].park.x][box[p].park.y] = 2;
		for (i=x1-1;i<=x2+1;i++)
            for (j=y1-1;j<=y2+1;j++)
                if (i<19 && j<19 && i>=2 && j>=2)
                    pista[i][j]=0;
        turnsEncoding(x1, y1-2, 11);
		turnsEncoding(x1+1, y1-2, 12);
		turnsEncoding(x1+2, y1-2, 13);
		
		turnsEncoding(x2+2, y1, 21);
		turnsEncoding(x2+2, y1+1, 22);
		turnsEncoding(x2+2, y1+2, 23);
		
		turnsEncoding(x2, y2+2, 31);
		turnsEncoding(x2-1, y2+2, 32);
		turnsEncoding(x2-2, y2+2, 33);
		
		turnsEncoding(x1-2, y2, 41);
		turnsEncoding(x1-2, y2-1, 42);
		turnsEncoding(x1-2, y2-2, 43);
		            
	}
		
    for (i=0;i<21;i++) {
    	printf("{");
        for (j=0;j<20;j++) {
            printf("%d,",pista[j][i]);
    	}
        printf("%d},\n",pista[j][i]);
	}
	int counter = 0;
	for (i=2;i<19;i++) {
        for (j=2;j<19;j++) {
			if (pista[i][j]!=0) { 
				counter++;
				pista[i][j] = counter;
				map[counter].x = i; 
				map[counter].y = j;
			}
		}
	}
	int V = counter+1;	
	unsigned short int graph[MAXV][MAXV]={0};
	int dist1 = round(sqrt(pow(CELL,2)+pow(CELL,2)));
	int dist2 = round(sqrt(pow(2*CELL,2)+pow(CELL,2)));
	int dist3 = round(sqrt(pow(3*CELL,2)+pow(CELL,2)));
	printf("dist1=%d dist2=%d dist3=%d\n",dist1,dist2,dist3);
	int node1, node2;
	
	for (i=2;i<19;i++) {
        for (j=2;j<19;j++) {
        	node1 = pista[i][j];
			if (node1!=0) {
				int xx,yy,flag;
				node2 = pista[i+1][j];
				if (node2!=0) {
					xx=i+1; yy=j; flag = checkTurnCode(xx,yy,41)+checkTurnCode(xx,yy,42)+checkTurnCode(xx,yy,43);
					if (flag==0)
						graph[node1][node2] = CELL;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i][j-1];
				if (node2!=0) {
					xx=i; yy=j-1; flag = checkTurnCode(xx,yy,31)+checkTurnCode(xx,yy,32)+checkTurnCode(xx,yy,33);
					if (flag==0)
						graph[node1][node2] = CELL;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-1][j];
				if (node2!=0) {
					xx=i-1; yy=j; flag = checkTurnCode(xx,yy,21)+checkTurnCode(xx,yy,22)+checkTurnCode(xx,yy,23);
					if (flag==0)
						graph[node1][node2] = CELL;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i][j+1];
				if (node2!=0) {
					xx=i; yy=j+1; flag = checkTurnCode(xx,yy,11)+checkTurnCode(xx,yy,12)+checkTurnCode(xx,yy,13);
					if (flag==0)
						graph[node1][node2] = CELL;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+2][j-1];
				if (node2!=0) {
					xx=i+2; yy=j-1; flag = checkTurnCode(xx,yy,41)+checkTurnCode(xx,yy,42)+checkTurnCode(xx,yy,43);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-2][j-1];
				if (node2!=0) {
					xx=i-2; yy=j-1; flag = checkTurnCode(xx,yy,31)+checkTurnCode(xx,yy,32)+checkTurnCode(xx,yy,33);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-2][j+1];
				if (node2!=0) {
					xx=i-2; yy=j+1; flag = checkTurnCode(xx,yy,21)+checkTurnCode(xx,yy,22)+checkTurnCode(xx,yy,23);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+2][j+1];
				if (node2!=0) {
					xx=i+2; yy=j+1; flag = checkTurnCode(xx,yy,11)+checkTurnCode(xx,yy,12)+checkTurnCode(xx,yy,13);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+1][j-1];
				if (node2!=0) {
					xx=i+1; yy=j-1; flag = checkTurnCode(xx,yy,42)+checkTurnCode(xx,yy,41)+checkTurnCode(xx,yy,33)+checkTurnCode(xx,yy,32);
					if (flag==0)
						graph[node1][node2] = dist1;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-1][j-1];
				if (node2!=0) {
					xx=i-1; yy=j-1; flag = checkTurnCode(xx,yy,32)+checkTurnCode(xx,yy,31)+checkTurnCode(xx,yy,23)+checkTurnCode(xx,yy,22);
					if (flag==0)
						graph[node1][node2] = dist1;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-1][j+1];
				if (node2!=0) {
					xx=i-1; yy=j+1; flag = checkTurnCode(xx,yy,22)+checkTurnCode(xx,yy,21)+checkTurnCode(xx,yy,13)+checkTurnCode(xx,yy,12);
					if (flag==0)
						graph[node1][node2] = dist1;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+1][j+1];
				if (node2!=0) {
					xx=i+1; yy=j+1; flag = checkTurnCode(xx,yy,12)+checkTurnCode(xx,yy,11)+checkTurnCode(xx,yy,43)+checkTurnCode(xx,yy,42);
					if (flag==0)
						graph[node1][node2] = dist1;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+1][j-2];
				if (node2!=0) {
					xx=i+1; yy=j-2; flag = checkTurnCode(xx,yy,31)+checkTurnCode(xx,yy,32)+checkTurnCode(xx,yy,33);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-1][j-2];
				if (node2!=0) {
					xx=i-1; yy=j-2; flag = checkTurnCode(xx,yy,21)+checkTurnCode(xx,yy,22)+checkTurnCode(xx,yy,23);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i-1][j+2];
				if (node2!=0) {
					xx=i-1; yy=j+2; flag = checkTurnCode(xx,yy,11)+checkTurnCode(xx,yy,12)+checkTurnCode(xx,yy,13);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
				node2 = pista[i+1][j+2];
				if (node2!=0) {
					xx=i+1; yy=j+2; flag = checkTurnCode(xx,yy,41)+checkTurnCode(xx,yy,42)+checkTurnCode(xx,yy,43);
					if (flag==0)
						graph[node1][node2] = dist2;
					else
						graph[node1][node2] = BIGDIST;
				}
			}
		}
	}
	
	int startNode = pista[parkArea.pathStart.x][parkArea.pathStart.y];
	int boxParkPointSrc, boxParkPointDest;
	int distance, d[5];
	int boxCheckOrder[4]={-1,-1,-1,-1};
	int checked[5]={0,0,0,0,0};
	for (i=0;i<5;i++) {
		d[i]= dijkstra(V, graph, startNode, pista[box[i].park.x][box[i].park.y]); 
		printf("%d ",d[i]);
	}
	int nearestBox = findPosOfMin(d,5);
	printf("\n\nNearest Box: %d , Route:",nearestBox);
	int w, index = 0;
	boxCheckOrder[index] = nearestBox;
	checked[nearestBox] = 1;
	
	for (w=1;w<=3;w++) {
		for (i=0;i<5;i++) {
			if (checked[i]==0)
			    d[i] = dijkstra(V, graph, pista[box[nearestBox].park.x][box[nearestBox].park.y], pista[box[i].park.x][box[i].park.y]);
			else 
				d[i] = 999999;     
		}
		nearestBox = findPosOfMin(d,5);
		index++;
		boxCheckOrder[index] = nearestBox;
		checked[nearestBox] = 1;
	} 
	
	for (i=0;i<4;i++) {
		w=9+9*boxCheckOrder[i];
		for (j=0;j<9;j++) 
		   printf("%c",QRcode[w+j]);			
	}

	boxParkPointDest = pista[box[boxCheckOrder[0]].park.x][box[boxCheckOrder[0]].park.y];

	robotMoves[0]='\0';
	printf("\n\nPARK -> 1st CUBE\n");
	dijkstra(V, graph, startNode, boxParkPointDest); 
	robotPath[robotPathCounter-1].dir = box[boxCheckOrder[0]].parkDir; 
	
	robot.center.x = parkArea.entryPoint.x;
	robot.center.y = parkArea.entryPoint.y;
	robot.dir = parkArea.dir;
	
	point_t temp;
	temp.x = round(parkArea.entryPoint.x + 100 * cos(toRadians(parkArea.dir)));
	temp.y = round(parkArea.entryPoint.y - 100 * sin(toRadians(parkArea.dir)));

	point_t parkExitPoint = calculateInterceptionPoint(parkArea.entryPoint, temp, robotPath[0].p, robotPath[1].p);
	
	printPoint(parkArea.entryPoint,"entryPoint-meso: ");
	printPoint(temp,"temp: ");
	printPoint(parkExitPoint,"parkExitPoint: ");
	javaInsertPoint(parkArea.entryPoint, "ORANGE", 8);
	javaInsertPoint(temp, "ORANGE", 8);
	javaInsertPoint(parkExitPoint, "BLUE", PAINNER);
	javaInsertPoint(robotPath[robotPathCounter-1].p, "BLUE", 8);
	
	
	distance = calcDistance(robot.center, parkExitPoint);
    distance = distance - calcDistanceOnRotationEllipse(); 
    moveBack(distance);
	robotPath[0].p = parkExitPoint;
	
	moveToNextBox();
	int x_temp,y_temp;
	for (w=1;w<=3;w++) {
		if (w==3) {
			x_temp = robot.center.x;
			y_temp = robot.center.y;
		}
		if (CALIB1==2) {
			sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
		} else if (CALIB1==0) {
			sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);
		} else if (w==2) {
			sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
		} else sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);
		
		sprintf(robotMoves,"%s%d,%d,",robotMoves,COLORCHECK,w);
		moveBackFromBox();
		printf("\n\nCUBE%d -> CUBE%d\n",w,w+1);
		boxParkPointSrc = boxParkPointDest;
		boxParkPointDest = pista[box[boxCheckOrder[w]].park.x][box[boxCheckOrder[w]].park.y];
		dijkstra(V, graph, boxParkPointSrc, boxParkPointDest); 
		robotPath[robotPathCounter-1].dir = box[boxCheckOrder[w]].parkDir;  
		moveToNextBox();
	}
	if (CALIB1==2) {
		sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
	} else if (CALIB1==0) {
		sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);
	} else {
		sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
	}
	sprintf(robotMoves,"%s%d,%d,",robotMoves,COLORCHECK,4);
	
	printf("\n%s\n", robotMoves);
   	r1 = fopen(route1, "w"); 
   	fprintf(r1, "%s\n", robotMoves);
	fclose(r1);
	robotMoves[0]='\0';
	
	printf("WAITING FOR 3 COLORS....\n");
	int found1 = 0;
	do {
		sleep(0.5);
		if( found1 == 0 && access( cubes3Colors, F_OK ) != -1 ) 
		    found1 = 1;
	} while (found1==0);
	printf("\n\nAWAKE!\n");
    sleep(1);
	c1 = fopen(cubes3Colors,"r");
	if (c1==NULL) printf("error reading cubes3Colors.txt!\n");
	int colorNum;
	int sumBoxColors=0, sumCubeColors=0, sumBoxIndex=0;
	int nextColor=-1;
	int cubes3OK=0;
	int lastCube;
	if (c1) {
		for (w=0;w<3;w++) {
			c = getc(c1);
			colorNum = (c-'0') * 10;
			c = getc(c1);
			colorNum = colorNum + (c-'0');
			box[boxCheckOrder[w]].boxColor = colorNum;
			sumBoxColors+=colorNum;
			c = getc(c1);
			c = getc(c1);
			colorNum = (c-'0') * 10;
			c = getc(c1);
			colorNum = colorNum + (c-'0');
			box[boxCheckOrder[w]].cubeColor = colorNum;
			sumCubeColors+=colorNum;
			sumBoxIndex+=boxCheckOrder[w];
			c = getc(c1);
		}
	  	fclose(c1);
	  	
		for (w=0;w<5;w++) {
			if (box[w].boxColor==BLUE) {
				cubes3OK++;
				protos = w;
				nextColor = box[w].cubeColor;
				break;
			}
		}
		for (w=0;w<5;w++) {
			if (box[w].boxColor==nextColor) {
				cubes3OK++;
				deyteros = w;
				nextColor = box[w].cubeColor;
				break;
			}
		}
		for (w=0;w<5;w++) {
			if (box[w].boxColor==nextColor) {
				cubes3OK++;
				tritos = w;
				break;
			}
		}		
		
	}
	cf = fopen(cubes3Flag,"w");
	if (cf==NULL) printf("error creating cubes3Flag.txt!\n");

	if (cubes3OK==3) {
   		fprintf(cf, "1\n");
   		fclose(cf);
   		printBoxesInOrder();
   		lastCube = boxCheckOrder[2];
   		robot.dir = box[lastCube].parkDir;
        robot.center.x = x_temp;
		robot.center.y = y_temp;
	} else {
		fprintf(cf, "0\n");
		fclose(cf);
		printf("WAITING FOR COLORS of 4th CUBE....\n");
		int found1 = 0;
		do {
			sleep(0.5);
			if( found1 == 0 && access( cubes4Colors, F_OK ) != -1 ) 
			    found1 = 1;
		} while (found1==0);
		printf("\n\nAWAKE!\n");
	    sleep(1);
	    
		c1 = fopen(cubes4Colors,"r");
		if (c1==NULL) printf("error reading cubes4Colors.txt!\n");
	
		if (c1) {
			w=3;
			c = getc(c1);
			colorNum = (c-'0') * 10;
			c = getc(c1);
			colorNum = colorNum + (c-'0');
			box[boxCheckOrder[w]].boxColor = colorNum;
			sumBoxColors+=colorNum;
			c = getc(c1); // ,
			c = getc(c1);
			colorNum = (c-'0') * 10;
			c = getc(c1);
			colorNum = colorNum + (c-'0');
			box[boxCheckOrder[w]].cubeColor = colorNum;
			sumCubeColors+=colorNum;
			sumBoxIndex+=boxCheckOrder[w];
			c = getc(c1);
	    	fclose(c1);
			int lastOne = 10-sumBoxIndex;
			box[lastOne].boxColor = 60 - sumBoxColors;
			box[lastOne].cubeColor = 60 - sumCubeColors; 
	
			for (w=0;w<5;w++) {
				if (box[w].boxColor==BLUE) {
					protos = w;
					nextColor = box[w].cubeColor;
					break;
				}
			}
			for (w=0;w<5;w++) {
				if (box[w].boxColor==nextColor) {
					deyteros = w;
					nextColor = box[w].cubeColor;
					break;
				}
			}
			for (w=0;w<5;w++) {
				if (box[w].boxColor==nextColor) {
					tritos = w;
					break;
				}
			}		
			printBoxesInOrder();
		}	
		lastCube = boxCheckOrder[3];
	}
    
	if (tritos != lastCube) { 
		printf("\n\nlast CUBE -> CUBE3\n LastCube=%d, its Dir:%d, %.1f, (%d,%d)",lastCube, box[lastCube].parkDir,robot.dir,robot.center.x,robot.center.y); fprintf(javaFile,"\n//last CUBE -> CUBE3\n");
		dijkstra(V, graph, pista[box[lastCube].park.x][box[lastCube].park.y], pista[box[tritos].park.x][box[tritos].park.y]); 
		robotPath[robotPathCounter-1].dir = box[tritos].parkDir; 
		moveBackFromBox();
		moveToNextBox();
		if (CALIB2==2)
		   sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
		else
		   sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);   
	}
	sprintf(robotMoves,"%s%d,%d,",robotMoves,TAKEANDDROP,0);
	moveBackFromBox();
	
	printf("\n\nCUBE3 -> CUBE2\n"); fprintf(javaFile,"\n//CUBE3 -> CUBE2\n");
	dijkstra(V, graph, pista[box[tritos].park.x][box[tritos].park.y], pista[box[deyteros].park.x][box[deyteros].park.y]); 
	robotPath[robotPathCounter-1].dir = box[deyteros].parkDir; 
	moveToNextBox();
	if (CALIB2==2)
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
	else
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);   
	sprintf(robotMoves,"%s%d,%d,",robotMoves,TAKEANDHOLD,0);
	moveBackFromBox();
	
	printf("\n\nCUBE2 -> CUBE3\n"); fprintf(javaFile,"\n//CUBE2 -> CUBE3\n");
	dijkstra(V, graph, pista[box[deyteros].park.x][box[deyteros].park.y], pista[box[tritos].park.x][box[tritos].park.y]); 
	robotPath[robotPathCounter-1].dir = box[tritos].parkDir; 
	moveToNextBox();
	if (CALIB2==0)
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);
	else
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);   
	sprintf(robotMoves,"%s%d,%d,",robotMoves,PLACECUBE,0);
	moveBackFromBox();
	
	printf("\n\nCUBE3 -> CUBE1\n"); fprintf(javaFile,"\n//CUBE3 -> CUBE1\n");
	dijkstra(V, graph, pista[box[tritos].park.x][box[tritos].park.y], pista[box[protos].park.x][box[protos].park.y]); 
	robotPath[robotPathCounter-1].dir = box[protos].parkDir; 
	moveToNextBox();
	if (CALIB2==2)
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);
	else
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);   
	sprintf(robotMoves,"%s%d,%d,",robotMoves,TAKEANDHOLD,0);
	moveBackFromBox();
	
	printf("\n\nCUBE1 -> CUBE2\n"); fprintf(javaFile,"\n//CUBE1 -> CUBE2\n");
	dijkstra(V, graph, pista[box[protos].park.x][box[protos].park.y], pista[box[deyteros].park.x][box[deyteros].park.y]); 
	robotPath[robotPathCounter-1].dir = box[deyteros].parkDir; 
	moveToNextBox();
	if (CALIB2==0)
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,NOCALIB,0);
	else
	   sprintf(robotMoves,"%s%d,%d,",robotMoves,CALIBRATE,0);   
	sprintf(robotMoves,"%s%d,%d,",robotMoves,PLACECUBE,0);
	moveBackFromBox();
	
	printf("\n\nCUBE2 -> PARKAREA\n"); fprintf(javaFile,"\n//CUBE2 -> PARKAREA\n");
	dijkstra(V, graph, pista[box[deyteros].park.x][box[deyteros].park.y], startNode); 
	robotPath[robotPathCounter-1].dir = parkArea.dir; 

	turnTo(robotPath[0].p,robotPath[0].dir);
 	for (i=1;i<robotPathCounter-1;i++) {
 		if (robotPath[i].dir==robot.dir && i<robotPathCounter-2) continue;
		distance = calcDistance(robot.center, robotPath[i].p);
	    distance = distance + calcDistanceOnRotationEllipse();
	    moveFront(distance);
	 	turnTo(robotPath[i].p,robotPath[i].dir); 		
	}

	point_t parkInterPoint = calculateInterceptionPoint(parkArea.entryPoint, temp, robotPath[robotPathCounter-2].p, robotPath[robotPathCounter-1].p);
	distance = calcDistance(robot.center, parkInterPoint);
    distance = distance + calcDistanceOnRotationEllipse();
    moveFront(distance);
 	turnTo(parkInterPoint,parkArea.dir);
	
	distance = calcDistance(robot.center, parkArea.entryPoint);
	distance = distance + 220 - DISTANCETOPARKFROMQR; 
	printf("\nrobot(%d,%d),entryPoint(%d,%d)%d\n",robot.center.x,robot.center.y,parkArea.entryPoint.x, parkArea.entryPoint.y,distance);
	
	javaInsertPoint(robot.center, "GREEN", 20);
	javaInsertPoint(parkArea.entryPoint, "GREEN", 20);
	javaInsertPoint(parkInterPoint, "BLUE", 20);
	printf("\nDISTANCE TO PARK: %d\n",distance);
	breakMovements(PARK,distance);
	sprintf(robotMoves,"%s%d,%d,",robotMoves,STOP,0);
	printf("\n%s\n", robotMoves);
	r2 = fopen(route2, "w"); 
   	fprintf(r2, "%s", robotMoves);
	fclose(r2);
	
	fclose(javaFile);
	
	return 0; 
}
