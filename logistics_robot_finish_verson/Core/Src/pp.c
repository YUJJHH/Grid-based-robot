#include "pp.h"

// cd C:\Users\yuuuj\Logistics_robot\window\build\Debug
// .\pp.exe


cell grid[ROW][COLUMN] = {0, };

cell startCell = {0, 0, IsEmpty, NULL, 0, 0};
cell endCell = {0, 0, IsEmpty, NULL, 0, 0};

cell cellDomain[CELL_DOMAIN_MAX] = {INIT_CELL, };
int CDHeader = 0;
int pathLength = 0;



void sort(){
    int i, j, least, n = CELL_DOMAIN_MAX;
    cell temp;

	for (i = 0; i < n - 1; i++) {

		least = i;

		for (j = i + 1; j < n; j++) 	// 최소값 탐색

			if (cellDomain[j].f < cellDomain[least].f) least = j;

		SWAP(cellDomain[i], cellDomain[least], temp);
	}
}

double clc_f(cell nowCell){
    double f, g, h;

    g = nowCell.g;
    h = abs_cust(nowCell.x-endCell.x)+abs_cust(nowCell.y-endCell.y);

    f = g+h*H_MUL;

    return f;
}

double abs_cust(double x){
    if(x < 0){
        return -x;
    }
    return x;
}

void pushCell(int x, int y, cell parent){
    cell* Cptr = &grid[x][y];
    Cptr->g = parent.g+1;
    Cptr->f = clc_f(*Cptr);
    Cptr->parentPtr = &grid[parent.x][parent.y];
    Cptr->state = IsPath;
    cellDomain[CDHeader++] = *Cptr;
}

void push4(cell middleCell){
    int x = middleCell.x, y = middleCell.y;

    // printf("x: %d, y: %d\n", x, y);
    //LRDU
    if(x > 0 && grid[x-1][y].state ==IsEmpty){
        pushCell(x-1, y, middleCell);
    }
    if(x < COLUMN-1 && grid[x+1][y].state == IsEmpty){
        pushCell(x+1, y, middleCell);
    }
    if(y > 0 && grid[x][y-1].state == IsEmpty){
        pushCell(x, y-1, middleCell);
    }
    if(y < ROW-1 && grid[x][y+1].state == IsEmpty){
        pushCell(x, y+1, middleCell);
    }
}

void gridInit(int state){
    int i =0, j =0;

    for(i =0; i<ROW; i++){
        for(j = 0; j<COLUMN; j++){
            grid[i][j].x = i;
            grid[i][j].y = j;
            grid[i][j].state = state;
            grid[i][j].parentPtr = NULL;
            grid[i][j].f = 0;
            grid[i][j].g = 0;
        }
    }
}

char setDpp(int scol, int srow, int ecol, int erow){
    if(scol >= ROW || scol < 0){
    //    fprintf(stderr,"scol out of bound!\n");
    }
    if(ecol >= ROW || ecol < 0){
      //  fprintf(stderr,"ecol out of bound!\n");
    }
    if(srow >= COLUMN || srow < 0){
     //   fprintf(stderr,"srow out of bound!\n");
    }
    if(erow >= COLUMN || erow < 0){
    //    fprintf(stderr,"erow out of bound!\n");
    }
    cell init =INIT_CELL;

    int CDHeader = 0, i=0;
    int pathLength = 0;
    for(i =0; i<CELL_DOMAIN_MAX; i++){
        cellDomain[i] = init;
    }

    cell scell = {srow, scol, IsPath, NULL, 0, 0};
    cell ecell = {erow, ecol, IsEmpty, NULL, 0, 0};

    startCell = scell;
    endCell = ecell;

    grid[srow][scol] = startCell;
    grid[erow][ecol] = endCell;

    if(startCell.state == IsOccpyed ){
        return IsOccpyed;
    }
    if(endCell.state == IsOccpyed ){
        return IsOccpyed;
    }
    return IsEmpty;
}

cell lpp(){
    cell init = INIT_CELL;
    cell tmp = INIT_CELL;

    do
    {
        if(CDHeader == 0){
            push4(startCell);
        }else{
            push4(tmp);
        }
        sort();
        tmp = cellDomain[0];
        if(tmp.f == DOMAIN_NULL){
            return init;
            printf("fail to lpp\n");
        }
        printf("x: %d y: %d f: %f\n", tmp.x, tmp.y, tmp.f);
        cellDomain[0] = init;
    } while (tmp.x != endCell.x || tmp.y != endCell.y);
    return tmp;
}

void readPath(cell tagCell, cell path[CELL_DOMAIN_MAX]){
    printf("\n\nRead Path\n");
    int pathIndex = 0, i=0;
    path[pathIndex++] = tagCell;
    printf("x: %d y: %d f: %f\n", path[0].x, path[0].y, path[0].f);
    cell* tmp = tagCell.parentPtr;
    while(tmp != NULL){
        printf("x: %d y: %d f: %f\n", tmp->x, tmp->y, tmp->f);
        path[pathIndex++] = *tmp;
        tmp = tmp->parentPtr;
    }
    pathLength = pathIndex;
}


/*
int main(){
    cell path[CELL_DOMAIN_MAX];

    gridInit(IsEmpty);
    setDpp(0, 0, 1, 2);
    readPath(lpp(), path);
    printf("path length: %d\n", pathLength);

    // printf("%d %d\n",endCell.x, endCell.y);

    // push4(startCell);

    // printf("%d %d header: %d f: %f g: %f\n"
    // , cellDomain[CDHeader-1].x, cellDomain[CDHeader-1].y, CDHeader
    // , cellDomain[CDHeader-1].f, cellDomain[CDHeader-1].g);

    // sort();

    // printf("%d %d header: %d f: %f g: %f\n"
    // , cellDomain[0].x, cellDomain[0].y, CDHeader
    // , cellDomain[0].f, cellDomain[0].g);


    // printf("%d %d\n", grid[0][1].x, grid[0][1].y);
    //cell endTag = lpp();

    //printf("%d %d\n", endTag.x, endTag.y);

    return 0;
}*/
