#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <string.h>

/* global parameters */
int RAND_SEED[] = {1,20,30,40,50,60,70,80,90,100,110, 120, 130, 140, 150, 160, 170, 180, 190, 200};
int NUM_OF_RUNS = 1;
int MAX_TIME = 30;  //max amount of time permitted (in sec)
int num_of_problems;
clock_t START_TIME, STOP_TIME;


/* parameters for VNS algorithms */
int NUMBER_OF_NEIGHBORS = 3;
int VNS_SWAP_NUM = 10000;

/* parameters for PSO algorithms */
int SWARM_SIZE=30;    // [20, 40]

// coefficient
double w=1;
float Cp=1;
float Cg=3;
float V_MAX=5;


struct solution_struct best_sln;  //global best solution

//return a random number between 0 and 1
float rand_01()
{
    float number;
    number = (float) rand();
    number = number/RAND_MAX;
    //printf("rand01=%f\n", number);
    return number;
}

//return a random number ranging from min to max (inclusive)
int rand_int(int min, int max)
{
    int div = max-min+1;
    int val =rand() % div + min;
    //printf("rand_range= %d \n", val);
    return val;
}


struct item_struct{
    int dim; //no. of dimensions
    int* size; //volume of item in all dimensions
    int p;
    double ratio;
    int index;
};

struct problem_struct{
    int n; //number of items
    int dim; //number of dimensions
    struct item_struct* items;
    int* capacities;  //knapsack capacities
};

void free_problem(struct problem_struct* prob)
{
    if(prob!=NULL)
    {
        if(prob->capacities !=NULL) free(prob->capacities);
        if(prob->items!=NULL)
        {
            for(int j=0; j<prob->n; j++)
            {
                if(prob->items[j].size != NULL)
                    free(prob->items[j].size);
            }
            free(prob->items);
        }
        free(prob);
    }
}

void init_problem(int n, int dim, struct problem_struct** my_prob)
{
    struct problem_struct* new_prob = malloc(sizeof(struct problem_struct));
    new_prob->n=n; new_prob->dim=dim;
    new_prob->items=malloc(sizeof(struct item_struct)*n);
    for(int j=0; j<n; j++)
        new_prob->items[j].size= malloc(sizeof(int)*dim);
    new_prob->capacities = malloc(sizeof(int)*dim);
    *my_prob = new_prob;
}


//example to create problem instances, actual date should come from file
struct problem_struct** load_problems(char* data_file)
{
    int i,j,k;
    //int num_of_probs;
    FILE* pfile = fopen(data_file, "r");
    if(pfile==NULL)
    {printf("Data file %s does not exist. Please check!\n", data_file); exit(2); }
    fscanf (pfile, "%d", &num_of_problems);

    struct problem_struct** my_problems = malloc(sizeof(struct problem_struct*)*num_of_problems);
    for(k=0; k<num_of_problems; k++)
    {
        int n, dim, obj_opt;
        fscanf (pfile, "%d", &n);
        fscanf (pfile, "%d", &dim);
//        fscanf (pfile, "%d", &obj_opt);
        fscanf (pfile, "%d", &obj_opt); // optimal solution value (zero if unavailable)

        init_problem(n, dim, &my_problems[k]);  //allocate data memory

        for(j=0; j<n; j++)
        {
            my_problems[k]->items[j].dim=dim;
            my_problems[k]->items[j].index=j;
            fscanf(pfile, "%d", &(my_problems[k]->items[j].p)); //read profit data
            //printf("item[j].p=%d\n",my_problems[k]->items[j].p);
        }
        for(i=0; i<dim; i++)
        {
            for(j=0; j<n; j++)
            {
                fscanf(pfile, "%d", &(my_problems[k]->items[j].size[i])); //read size data
                //printf("my_problems[%i]->items[%i].size[%i]=%d\n",k,j,i,my_problems[k]->items[j].size[i]);
            }
        }
        for(i=0; i<dim; i++){
            fscanf(pfile, "%d", &(my_problems[k]->capacities[i]));
            //printf("capacities[i]=%d\n",my_problems[k]->capacities[i] );
        }
    }
    fclose(pfile); //close file
    return my_problems;
}

struct solution_struct{
    struct problem_struct* prob; //maintain a shallow copy of the problem data
    float objective;
    int feasibility; //indicate the feasiblity of the solution
    int* x; //chromosome vector
    int* cap_left; //capacity left in all dimensions
    // struct solution_struct* global_best;
    struct solution_struct* personal_best;
    float* v;   // velocity
};

void free_solution(struct solution_struct* sln)
{
    if(sln!=NULL)
    {
        free(sln->x);
        free(sln->cap_left);
        sln->objective=0;
        sln->prob=NULL;
        sln->feasibility=false;
    }
}

//copy a solution from another solution
bool copy_solution(struct solution_struct* dest_sln, struct solution_struct* source_sln)
{
    if(source_sln ==NULL) return false;
    if(dest_sln==NULL)
    {
        dest_sln = malloc(sizeof(struct solution_struct));
    }
    else{
        free(dest_sln->cap_left);
        free(dest_sln->x);
    }
    int n = source_sln->prob->n;
    int m =source_sln->prob->dim;
    dest_sln->x = malloc(sizeof(int)*n);
    dest_sln->cap_left=malloc(sizeof(int)*m);
    for(int i=0; i<m; i++)
        dest_sln->cap_left[i]= source_sln->cap_left[i];
    for(int j=0; j<n; j++)
        dest_sln->x[j] = source_sln->x[j];
    dest_sln->prob= source_sln->prob;
    dest_sln->feasibility=source_sln->feasibility;
    dest_sln->objective=source_sln->objective;
    return true;
}



void evaluate_solution(struct solution_struct* sln)
{
    //evaluate the feasibility and objective of the solution
    sln->objective =0; sln->feasibility = 1;
    struct item_struct* items_p = sln->prob->items;

    for(int i=0; i< items_p->dim; i++)
    {
        sln->cap_left[i]=sln->prob->capacities[i];
        for(int j=0; j<sln->prob->n; j++)
        {
            sln->cap_left[i] -= items_p[j].size[i]*sln->x[j];
            if(sln->cap_left[i]<0) {
                sln->feasibility = -1*i; //exceeding capacity
                return;
            }
        }
    }
    if(sln->feasibility>0)
    {
        for(int j=0; j<sln->prob->n; j++)
        {
            sln->objective += (float)sln->x[j] * (float)items_p[j].p;
        }
    }
}

//output a given solution to a file
void output_solution(struct solution_struct* sln, char* out_file)
{
    if(out_file !=NULL){
        FILE* pfile = fopen(out_file, "a"); //append solution data
        fprintf(pfile, "%i\n", (int)sln->objective);
        for(int i=0; i<sln->prob->n; i++)
        {
            fprintf(pfile, "%i ", sln->x[i]);
        }
        fprintf(pfile, "\n");
        /*for(int j=0; j<sln->prob->n; j++)
            fprintf(pfile, "%i ", sln->prob->items[j].p);
        fprintf(pfile, "\n");*/
        fclose(pfile);
    }
    else
        printf("sln.feas=%d, sln.obj=%f\n", sln->feasibility, sln->objective);
}


//check the  feasiblity and obj values of solutons from solution_file.
//return 0 is all correct or the index of the first infeasible problem [1, num_of_problems].
int check_solutions(struct problem_struct** my_problems, char* solution_file)
{
    FILE * pfile= fopen(solution_file, "r");
    if(pfile==NULL)
    {
        printf("Solution file %s does not exist. Please check!\n", solution_file);
        exit(2);
    }
    float val_obj;
    int val;
    fscanf (pfile, "%i", &val);
    if(val != num_of_problems)
    {
        printf("The stated number of solutions does not match the number of problems.\n");
        exit(3);
    }
    struct solution_struct temp_sln;
    int count=0, k=0;
    int n, dim;
    while(fscanf (pfile, "%f", &val_obj)!=EOF && k<num_of_problems)
    {
        //val_obj = val;
        n= my_problems[k]->n;  dim= my_problems[k]->dim;
        temp_sln.x = malloc(sizeof(int)*n);
        temp_sln.cap_left=malloc(sizeof(int)*dim);
        temp_sln.prob = my_problems[k];
        while(fscanf (pfile, "%i", &val)!=EOF)
        {
            if(val<0 || val>1) {fclose(pfile);  return k+1;} //illeagal values
            temp_sln.x[count] = val;
            count++;
            if(count==n)
            {
                evaluate_solution(&temp_sln);
                if(!temp_sln.feasibility || fabs(temp_sln.objective - val_obj)>0.01)
                {
                    fclose(pfile);
                    //printf("feasb=%i, obj= %f, val=%i\n",temp_sln.feasibility, temp_sln.objective, val_obj);
                    //output_solution(&temp_sln, "my_debug.txt");
                    return k+1;  //infeasible soltuion or wrong obj
                }
                else{
                    break;
                }
            }
        }
        count=0; k++;

        free(temp_sln.x); free(temp_sln.cap_left);
    }
    fclose(pfile);
    return 0;
}

int cmpfunc1(const void* a, const void* b){
    const struct item_struct* item1 = a;
    const struct item_struct* item2 = b;
    if(item1->ratio>item2->ratio) return -1;
    if(item1->ratio<item2->ratio) return 1;
    return 0;
}
int cmpfunc2 (const void * a, const void * b) {
    const struct item_struct* item1 = a;
    const struct item_struct* item2 = b;
    if(item1->index > item2->index) return 1;
    if(item1->index < item2->index) return -1;
    return 0;
}

void check_feasibility_afterSort(struct solution_struct* pop) {
    pop->feasibility = 1;
    struct item_struct* items_p = pop->prob->items;

    for(int i=0; i< items_p->dim; i++)
    {
        pop->cap_left[i]=pop->prob->capacities[i];
        for(int j=0; j<pop->prob->n; j++)
        {
            pop->cap_left[i] -= items_p[j].size[i]*pop->x[pop->prob->items[j].index];
            if(pop->cap_left[i]<0) {
                pop->feasibility = -1*i; //exceeding capacity
            }
        }
    }
}

void check_feasibility(struct solution_struct* pop) {
    pop->feasibility = 1;
    struct item_struct* items_p = pop->prob->items;

    for(int i=0; i< items_p->dim; i++)
    {
        pop->cap_left[i]=pop->prob->capacities[i];
        for(int j=0; j<pop->prob->n; j++)
        {
            pop->cap_left[i] -= items_p[j].size[i]*pop->x[j];
            if(pop->cap_left[i]<0) {
                pop->feasibility = -1*i; //exceeding capacity
            }
        }
    }
}


void check_feasibility2(bool ratio, struct solution_struct* pop) {
    pop->feasibility = 1;
    struct item_struct* items_p = pop->prob->items;

    for(int i=0; i< items_p->dim; i++)
    {
        pop->cap_left[i]=pop->prob->capacities[i];
        for(int j=0; j<pop->prob->n; j++)
        {
            if(ratio==true) {
                pop->cap_left[i] -= items_p[j].size[i]*pop->x[pop->prob->items[j].index];
            } else {
                pop->cap_left[i] -= items_p[j].size[i]*pop->x[j];
            }
            if(pop->cap_left[i]<0) {
                pop->feasibility = -1; //exceeding capacity
            }
        }
    }
}


void feasibility_repair2(struct solution_struct* slt) {
    // sort the items with the ratio
    for(int i=0; i<slt->prob->n;i++) {
        double avg_size=0;
        struct item_struct* item_i = &slt->prob->items[i];
        for(int d=0; d< slt->prob->dim; d++){
            avg_size += (double)item_i->size[d]/slt->prob->capacities[d];
        }
        item_i->ratio = item_i->p/avg_size;
    }
    qsort(slt->prob->items, slt->prob->n, sizeof(struct item_struct), cmpfunc1);

    // check the feasiblity of the best solution
    // gain more profit if it is already feasible, else reduce the items to make it feasible
    check_feasibility_afterSort(slt);
    if(slt->feasibility == 1) { // if it not exceed the capacity
        for(int j = 0; j < slt->prob->n; j++){
            if(slt->x[slt->prob->items[j].index] == 0) {
                slt->x[slt->prob->items[j].index] = 1;
                check_feasibility_afterSort(slt);
                if(slt->feasibility == -1) {
                    slt->x[slt->prob->items[j].index] = 0;
                    check_feasibility_afterSort(slt);
                    break;
                }
            }
        }
    }else { // if it exceed the capacity
        for(int j = (slt->prob->n)-1; j >= 0; j--){
            if(slt->x[slt->prob->items[j].index] == 1) {
                slt->x[slt->prob->items[j].index] = 0;
                check_feasibility_afterSort(slt);
                if(slt->feasibility == 1) {
                    break;
                }
            }
        }
    }

    // sort the items with the index
    qsort(slt->prob->items, slt->prob->n, sizeof(struct item_struct), cmpfunc2);

    // update the feasibility and the objective
    check_feasibility(slt);
    slt->objective=0;
    for(int m=0; m<slt->prob->n; m++) {
        slt->objective += (float)slt->x[m] * (float)slt->prob->items[m].p;
    }

    if(slt->feasibility<=0) {
        printf("in repair\n");
    }
}
//modify the solutions that violate the capacity constraints
void feasibility_repair(struct solution_struct* pop) {
    for(int i=0; i<pop->prob->n;i++){
        double avg_size=0;
        struct item_struct* item_i = &pop->prob->items[i];
        for(int d=0; d< pop->prob->dim; d++){
            avg_size += (double)item_i->size[d]/pop->prob->capacities[d];
        }
        item_i->ratio = item_i->p/avg_size;
    }

//    printf("++++++++++++++++++++++++++++++++++++++++++++++\n");
//    for(int m=0; m<pop->prob->n; m++) {
//        printf("%d", pop->x[m]);
//        printf("+%f ", pop->prob->items->ratio);
//    }
//    printf("\n");
    qsort(pop->prob->items, pop->prob->n, sizeof(struct item_struct), cmpfunc1);


//    check_feasibility2(true, pop);
    check_feasibility_afterSort(pop);

    if(pop->feasibility<=0) {
        for(int i=(pop->prob->n)-1; i>=0;i--){
            if(pop->x[pop->prob->items[i].index] == 1) {
                pop->x[pop->prob->items[i].index] = 0;
//                check_feasibility2(true, pop);
                check_feasibility_afterSort(pop);
                if(pop->feasibility>0) {
                    break;
                }
            }
        }
    } else {
        for(int i=0; i<pop->prob->n;i++){
            if(pop->x[pop->prob->items[i].index] == 0) {
                pop->x[pop->prob->items[i].index] = 1;
//                check_feasibility2(true, pop);
                check_feasibility_afterSort(pop);
                if(pop->feasibility<=0) {
                    pop->x[pop->prob->items[i].index] = 0;
                    break;
                }
            }
        }
    }

//    check_feasibility_afterSort(pop);
//    printf("%d", pop->feasibility);

    qsort(pop->prob->items, pop->prob->n, sizeof(struct item_struct), cmpfunc2);
//    check_feasibility2(pop);
//    if(pop->feasibility<0) {
//        printf("%d", pop->feasibility);
//    }
//    for(int m=0; m<pop->prob->n; m++) {
//        printf("%d", pop->prob->items[m].index);
//    }
//    printf("------------------------------------------\n");
    pop->objective=0;
    for(int m=0; m<pop->prob->n; m++) {
        pop->objective += (float)pop->x[m] * (float)pop->prob->items[m].p;
    }
    check_feasibility(pop);
    if(pop->feasibility<=0) {
        printf("in repair");
    }
//    printf("feasibility: %d\n", pop->feasibility);
}



//update global best solution from sln
void update_best_solution(struct solution_struct* sln)
{
    if(best_sln.objective < sln->objective)
        copy_solution(&best_sln, sln);
}
struct solution_struct* greedy_heuristic(struct problem_struct* prob) {
    for(int i=0; i<prob->n;i++){
        double avg_size=0;
        struct item_struct* item_i = &prob->items[i];
        for(int d=0; d< prob->dim; d++){
            avg_size += (double)item_i->size[d]/prob->capacities[d];
        }
        item_i->ratio = item_i->p/avg_size;
    }
    qsort(prob->items, prob->n, sizeof(struct item_struct), cmpfunc1);

    struct solution_struct* init_sln = malloc(sizeof(struct solution_struct));
    init_sln->prob=prob;    init_sln->objective =0;
    init_sln->x = malloc(sizeof(int)*prob->n);
    init_sln->cap_left = malloc(sizeof(int)*prob->dim);
    int* cap = malloc(sizeof(int)*prob->dim);
    int i=0, d=0;
    for(d=0; d<prob->dim; d++) cap[d]=0; //aggregated volume
    for(i=0; i<prob->n; i++)
    {
        struct item_struct* item_i = &prob->items[i];
        //printf("item[%d].ratio = %.3f\t",item_i->indx,prob->items[i].ratio);
        for(d=0; d<prob->dim; d++){
            if(cap[d] + item_i->size[d] > prob->capacities[d])
                break; //infeasible to pack this item, try next
        }
        if(d>=prob->dim){
            init_sln->x[item_i->index] = 1;
            init_sln->objective += (float)item_i->p;
            for(d=0; d<prob->dim; d++){
                cap[d] += item_i->size[d];
            }
            //printf("packing item %d\n", item_i->indx);
        }
        else init_sln->x[item_i->index] =0;
    }
    for(d=0; d<prob->dim; d++){
        init_sln->cap_left[d] = prob->capacities[d]- cap[d];
    }
    free(cap);
    //restore item original order by sorting by index.
    qsort(prob->items, prob->n, sizeof(struct item_struct), cmpfunc2);

    evaluate_solution(init_sln);
    //output_solution(init_sln, "greedy_sln.txt");
    //printf("Init_sln obj=\t%d\tfeasiblity = %d.\n", init_sln->objective, init_sln->feasibility);
    return init_sln;
}

bool can_move(int nb_index, const int* move, struct solution_struct* pop ){
    if(nb_index == 1){
        for(int m =0; m < pop->prob->dim; m++)
        {
            if(pop->cap_left[m] + pop->prob->items[move[0]].size[m] < pop->prob->items[move[1]].size[m]) {
                return false;
            }
        }
    } else if(nb_index == 2) {
        //1-2 swap
        int i= move[0], j= move[1], l= move[2];
        if(i<0 || j<0 || l < 0) {
            return false;
        }
        for(int d=0; d < pop->prob->dim; d++){
            if(pop->cap_left[d] + pop->prob->items[i].size[d] < pop->prob->items[j].size[d] + pop->prob->items[l].size[d]) {
                return false;
            }
        }
    } else if (nb_index == 3) {
        //2-1 swap
        int i= move[0], j= move[1], l= move[2];
        if(i<0 || j<0 || l < 0) {
            return false;
        }
        for(int d=0; d < pop->prob->dim; d++){
            if(pop->cap_left[d] + pop->prob->items[i].size[d] + pop->prob->items[j].size[d] < pop->prob->items[l].size[d]) {
                return false;
            }
        }
    } else {
        return false;
    }

    return true;
}

struct solution_struct* best_descent_11(struct solution_struct* pop) {
    struct solution_struct* best_new = malloc(sizeof(struct solution_struct));
    best_new->cap_left = malloc(sizeof(int) * pop->prob->dim);
    best_new->x = malloc(sizeof(int) * pop->prob->n);
    copy_solution(best_new, pop);
    int n=pop->prob->n;
    int delta=0, best_delta=0;  //storing best neighbourhood moves
    int item1, item2;
    int b_item1 = -1, b_item2 = -1;

    int ones[n], zeros[n];
    int ones_index = 0;
    int zeros_index = 0;

    //pair swap
    for(int i = 0; i < n; i++){
        if(pop->x[i] == 1){
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    for(int i=0; i < ones_index; i++){
        for(int j=0; j < zeros_index; j++){
            item1 = ones[i];
            item2 = zeros[j];

            if(can_move(1, &item1, best_new)){
                delta = pop->prob->items[item2].p - pop->prob->items[item1].p;
                if(delta > best_delta){
                    best_delta = delta;
                    b_item1 = item1;
                    b_item2 = item2;
                }
            }
        }
    }
    if(best_delta>0) {
        for(int d=0; d<best_new->prob->dim; d++){
            best_new->cap_left[d] = best_new->cap_left[d] + best_new->prob->items[b_item1].size[d]-
                                    best_new->prob->items[b_item2].size[d];
        }
        best_new->x[b_item1]=0;
        best_new->x[b_item2]=1;
        best_new->objective = best_new->objective + (float)best_delta;
    }
    return best_new;
}

struct solution_struct* best_descent_12(struct solution_struct* pop) {
    struct solution_struct* best_new = malloc(sizeof(struct solution_struct));
    best_new->cap_left = malloc(sizeof(int) * pop->prob->dim);
    best_new->x = malloc(sizeof(int) * pop->prob->n);
    copy_solution(best_new, pop);
    int n=pop->prob->n;
    int delta=0, best_delta=0;  //storing best neighbourhood moves
    int item1, item2, item3;
    int b_item1=-1, b_item2=-1, b_item3=-1;

    int ones[n], zeros[n];
    int ones_index = 0;
    int zeros_index = 0;
    int iteration = 0;

    // 1-2 swap
    for(int i = 0; i < n; i++){
        if(pop->x[i] == 1){
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    while(iteration < VNS_SWAP_NUM){
        int i = rand_int(0, ones_index - 1);
        int j = rand_int(0, zeros_index - 1);
        int l = rand_int(0, zeros_index - 1);
        while(j == l){
            l = rand_int(0, zeros_index - 1);
        }
        item1=ones[i];
        item2=zeros[j];
        item3=zeros[l];
        if(can_move(2, &item1, best_new)){
            delta = pop->prob->items[item3].p + pop->prob->items[item2].p - pop->prob->items[item1].p;
            if(delta > best_delta){
                best_delta = delta;
                b_item1 = item1;
                b_item2 = item2;
                b_item3 = item3;
            }
        }
        iteration++;
    }
    if(best_delta>0) {
        for(int d=0; d<best_new->prob->dim; d++){
            best_new->cap_left[d] = best_new->cap_left[d]+best_new->prob->items[b_item1].size[d] -
                                    best_new->prob->items[b_item2].size[d] - best_new->prob->items[b_item3].size[d];
        }
        best_new->objective = best_new->objective + (float)best_delta;
        best_new->x[b_item1]=0;
        best_new->x[b_item2]=1;
        best_new->x[b_item3]=1;
    }
    return best_new;
}

struct solution_struct* best_descent_21(struct solution_struct* curt_sln) {
    struct solution_struct* best_new = malloc(sizeof(struct solution_struct));
    best_new->cap_left = malloc(sizeof(int) * curt_sln->prob->dim);
    best_new->x = malloc(sizeof(int) * curt_sln->prob->n);
    copy_solution(best_new, curt_sln);
    int n=curt_sln->prob->n;
    int delta=0, best_delta=0;  //storing best neighbourhood moves
    int item1, item2, item3;
    int b_item1=-1, b_item2=-1, b_item3=-1;

    int ones[n], zeros[n];
    int ones_index = 0;
    int zeros_index = 0;
    int iteration = 0;

    // 2-1 swap
    for(int i = 0; i < n; i++){
        if(curt_sln->x[i] == 1){
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    while(iteration < VNS_SWAP_NUM){
        int i = rand_int(0, ones_index - 1);
        int j = rand_int(0, ones_index - 1);
        int l = rand_int(0, zeros_index - 1);
        while(j == i){
            j = rand_int(0, ones_index - 1);
        }
        item1=ones[i];
        item2=ones[j];
        item3=zeros[l];
        if(can_move(3, &item1, best_new)){
            delta = curt_sln->prob->items[item3].p - curt_sln->prob->items[item1].p - curt_sln->prob->items[item2].p;
            if(delta > best_delta){
                best_delta = delta;
                b_item1 = item1;
                b_item2 = item2;
                b_item3 = item3;
            }
        }
        iteration++;
    }
    if(best_delta>0) {
        for(int d=0; d<best_new->prob->dim; d++){
            best_new->cap_left[d] = best_new->cap_left[d]+best_new->prob->items[b_item1].size[d] +
                                    best_new->prob->items[b_item2].size[d] - best_new->prob->items[b_item3].size[d];
        }
        best_new->objective = best_new->objective + (float)best_delta;
        best_new->x[b_item1]=0;
        best_new->x[b_item2]=0;
        best_new->x[b_item3]=1;
    }
    return best_new;
}


struct solution_struct* select_neighbor(int neighbor_index, struct solution_struct* pop) {
    struct solution_struct* best_new = malloc(sizeof(struct solution_struct));
    best_new->cap_left = malloc(sizeof(int) * pop->prob->dim);
    best_new->x = malloc(sizeof(int) * pop->prob->n);
    copy_solution(best_new, pop);

    if(neighbor_index == 1) {
        best_new=best_descent_11(pop);
    } else if (neighbor_index == 2){
        best_new=best_descent_12(pop);
    } else if (neighbor_index == 3) {
        best_new=best_descent_21(pop);
    }

    return best_new;
}

void VNS_search(struct solution_struct* pop){
    for(int i = 0; i < SWARM_SIZE; i++){
        int nb_index = 0; //neighborhood index
        STOP_TIME=clock();
        double time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
        struct solution_struct* curt_sln = &pop[i];

        while(time_spent < MAX_TIME && nb_index<NUMBER_OF_NEIGHBORS){
            struct solution_struct* neighbor = select_neighbor(nb_index + 1, curt_sln);
            if(neighbor->objective > curt_sln->objective){
                copy_solution(curt_sln, neighbor);
                nb_index=1;
            } else {
                nb_index++;
            }

            int idx = rand_int(0, (curt_sln->prob->n)-1);
//            if(curt_sln->x[idx] == 0) {
//                curt_sln->x[idx] = 1;
//                check_feasibility2(false, curt_sln);
//                if(curt_sln->feasibility < 0) {
//                    curt_sln->x[idx] = 0;
//                }
//            } else {
//                curt_sln->x[idx] = 0;
//            }

            free_solution(neighbor);
            free(neighbor);
        }

        pop[i] = *curt_sln;
        feasibility_repair(&pop[i]);

        if(pop[i].objective>pop[i].personal_best->objective) {
            copy_solution(pop[i].personal_best, &pop[i]);
        }
        update_best_solution(&pop[i]);

        check_feasibility(&pop[i]);
        if(pop[i].feasibility<=0) {
            printf("in VNS\n");
        }
    }
}
void update(struct solution_struct* swarm){
    for(int i=0; i<SWARM_SIZE; i++) {
        swarm[i].objective = 0;   // before update each particle, clear its objective value
        for(int j=0; j<swarm->prob->n; j++) {
            float pp = rand_01();
            float pg = rand_01();

            // update velocity
            swarm[i].v[j] = (float)w*swarm[i].v[j] + pp*Cp*(float)(swarm[i].personal_best->x[j]-swarm[i].x[j]) + pg*Cg*(float)(best_sln.x[j]-swarm[i].x[j]);


            if(swarm[i].v[j]>V_MAX) {
                swarm[i].v[j]=V_MAX;
            }
            if(swarm[i].v[j]<-V_MAX) {
                swarm[i].v[j]=-V_MAX;
            }

            // sigmoid type function, move to new position
            float random_number = rand_01();
            if(random_number>1/(1+exp(-swarm[i].v[j]))) {
                swarm[i].x[j]=0;
            } else {
                swarm[i].x[j]=1;
            }

            // recalculate the objective
            swarm[i].objective += (float)swarm[i].x[j] * (float)swarm[i].prob->items[j].p;
//            fitness_calculate(&swarm[i]);
        }
        // repair
        feasibility_repair(&swarm[i]);
        //check feasibility
//        check_feasibility2(false, &swarm[i]);
        check_feasibility(&swarm[i]);
        if(swarm[i].feasibility<0) {
            printf("infeasible particle %d in update position\n", i);
        }
    }

    for(int i=0; i<SWARM_SIZE; i++) {
        // update personal best
        if(swarm[i].objective>swarm[i].personal_best->objective) {
            copy_solution(swarm[i].personal_best, &swarm[i]);
//            swarm[i].personal_best=&swarm[i];
        }

        //update global best
        update_best_solution(&swarm[i]);

        check_feasibility(&swarm[i]);
        if(swarm[i].feasibility<=0) {
            printf("in update\n");
        }

    }


//    update_global_best(swarm);
}



/**
 * initialize particle swarm, give initial position, velocity, personal best and global best
 * ignore the feasibility of each particle
 *
 * And finally repair them
 */
void initialize_particle_swarm(struct problem_struct* prob, struct solution_struct* sln) {
    for(int i=0; i<SWARM_SIZE; i++) {
        // malloc spaces
        sln[i].objective = 0;
        sln[i].prob = prob;
        sln[i].x = malloc(sizeof(int)*prob->n);
        sln[i].cap_left = malloc(sizeof(int)*prob->dim);
        sln[i].v = malloc(sizeof(float )*prob->n);
        sln[i].personal_best = malloc(sizeof(struct solution_struct));
        sln[i].personal_best->cap_left = malloc(sizeof(int)*sln[i].prob->dim);
        sln[i].personal_best->x = malloc(sizeof(int)*sln[i].prob->n);

        // initially assign particle itself as personal best
        copy_solution(sln[i].personal_best, &sln[i]);

        // find the big one as global best by using greedy search
        struct solution_struct* curt_sln = greedy_heuristic(prob);
        update_best_solution(curt_sln);

        // initialize position and velocity randomly for each particle
        for(int j=0; j<prob->n; j++) {
            sln[i].x[j] = rand()%2;
            sln[i].objective += (float)sln[i].x[j] * (float)sln[i].prob->items[j].p;
            sln[i].v[j] = rand_01();
        }

        // repair feasibility of each particle
        feasibility_repair(&sln[i]);
        // check feasibility
//        check_feasibility2(false, &sln[i]);
        check_feasibility(&sln[i]);
        if(sln[i].feasibility<0) {
            printf("infeasible particle %d in initialize_particle_swarm\n", i);
        }
    }
}



int PSO(struct problem_struct* prob) {
    START_TIME = clock();
    double time_spent=0;
    int iter =0;

    // create a particle swarm with size SWARM_SIZE
    struct solution_struct particle_swarm[SWARM_SIZE];
    // initialize the particle swarm
    initialize_particle_swarm(prob, particle_swarm);

    while(time_spent < MAX_TIME) {
        update(particle_swarm);
        check_feasibility(&best_sln);
        if(best_sln.feasibility<=0) {
            printf("before VNS\n");
        }
        VNS_search(particle_swarm);
        check_feasibility(&best_sln);
        if(best_sln.feasibility<=0) {
            printf("after VNS\n");
        }
        iter++;
        STOP_TIME=clock();
        time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
    }

    if(best_sln.feasibility<=0) {
        feasibility_repair(&best_sln);
    }
//    check_solutions()
//    check_feasibility2(false, &best_sln);
    check_feasibility(&best_sln);
    if(best_sln.feasibility<0) {
        printf("global best is not feasible, before print\n");
    }
//    printf("best: %f iter: %d time: %f\n", best_sln.objective, iter, time_spent);
    printf("%f\n", best_sln.objective);
//    for(int i=0; i<SWARM_SIZE; i++) {
//        free_solution(&particle_swarm[i]);
//        free(&particle_swarm[i]);
//    }

    return 0;

}



int main(int argc, const char * argv[]) {

    printf("Starting the run!\n");
    char data_file[50]={"somefile"}, out_file[50]={}, solution_file[50]={};  //max 50 problem instances per run
    if(argc<3)
    {
        printf("Insufficient arguments. Please use the following options:\n   -s data_file (compulsory)\n   -o out_file (default my_solutions.txt)\n   -c solution_file_to_check\n   -t max_time (in sec)\n");
        return 1;
    }
    else if(argc>9)
    {
        printf("Too many arguments.\n");
        return 2;
    }
    else
    {
        for(int i=1; i<argc; i=i+2)
        {
            if(strcmp(argv[i],"-s")==0)
                strcpy(data_file, argv[i+1]);
            else if(strcmp(argv[i],"-o")==0)
                strcpy(out_file, argv[i+1]);
            else if(strcmp(argv[i],"-c")==0)
                strcpy(solution_file, argv[i+1]);
            else if(strcmp(argv[i],"-t")==0)
                MAX_TIME = atoi(argv[i+1]);
        }
    }
    struct problem_struct** my_problems = load_problems(data_file);

    if(strlen(solution_file)<=0)
    {
        if(strcmp(out_file,"")==0) strcpy(out_file, "my_solutions.txt"); //default output
        FILE* pfile = fopen(out_file, "w"); //open a new file
        fprintf(pfile, "%d\n", num_of_problems); fclose(pfile);
        for(int k=0; k<num_of_problems; k++)
        {
            best_sln.objective=0; best_sln.feasibility=0;
            for(int run=0; run<NUM_OF_RUNS; run++)
            {
                srand(RAND_SEED[run]);
                PSO(my_problems[k]);   // call PSO method
            }
             output_solution(&best_sln,out_file);
        }
    }
    for(int k=0; k<num_of_problems; k++)
    {
        free_problem(my_problems[k]); //free problem data memory
    }
    free(my_problems); //free problems array
    if(best_sln.x!=NULL && best_sln.cap_left!=NULL){ free(best_sln.cap_left); free(best_sln.x);} //free global
    return 0;
}