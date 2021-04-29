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


//int K= 2; // k-opt is used
int VNS_SWAP_NUM = 10000;

/* parameters for PSO algorithms */
int SWARM_SIZE=100;    // [20, 40]
int MAX_NUM_OF_ITER = 200;
// range of particle's velocity and position
//const float V_MAX=3;
float P = 100;


// coefficient
//double Cp = 0.5;   // <=2
//double Cg = 2.5;   // <=2
//double w = 0.8;  // [0.4, 0.9]

double Cp_i = 2.5;   // <=2
double Cp_f = 0.5;
double Cg_f = 2.5;   // <=2
double Cg_i = 0.5;
double w_max = 1;  // [0.4, 0.9]
double w_min = 0.2;

struct solution_struct best_sln;  //global best solution
struct solution_struct next_best_sln;  //global best solution
struct solution_struct average_pb_sln;  //average personal best solution



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
    float fitness;
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
//pop->x[pop->prob->items[j].index]
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
// compute the fitness of each solution
void fitness_calculate(struct solution_struct* sln) {
    sln->fitness=0;
    //poslin x
    struct item_struct* items_p = sln->prob->items;
    int total_size_particle = 0;
    int total_size_package = 0;
    int penalty;
    for(int i=0; i< items_p->dim; i++)
    {
        sln->cap_left[i]=sln->prob->capacities[i];
        for(int j=0; j<sln->prob->n; j++)
        {
            total_size_particle = total_size_particle+items_p[j].size[i]*sln->x[j];
            total_size_package = total_size_package + sln->cap_left[i];
        }
    }
    int x = total_size_particle-total_size_package;
    if(x<0) {
        penalty=0;
    } else {
        penalty=x;
    }

    sln->fitness = sln->objective - P*(float)penalty;
}

//modify the solutions that violate the capacity constraints
void feasibility_repair(struct solution_struct* pop)
{
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


    check_feasibility_afterSort(pop);

    if(pop->feasibility<0) {
        for(int i=(pop->prob->n)-1; i>=0;i--){
            if(pop->x[pop->prob->items[i].index] == 1) {
                pop->x[pop->prob->items[i].index] = 0;
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
                check_feasibility_afterSort(pop);
                if(pop->feasibility<0) {
                    pop->x[pop->prob->items[i].index] = 0;
                    check_feasibility_afterSort(pop);
                    break;
                }
            }
        }
    }

//    check_feasibility_afterSort(pop);
//    printf("%d", pop->feasibility);

    qsort(pop->prob->items, pop->prob->n, sizeof(struct item_struct), cmpfunc2);
//    check_feasibility(pop);
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
//    printf("feasibility: %d\n", pop->feasibility);
}



//update global best solution from sln
void update_best_solution(struct solution_struct* sln)
{
    if(best_sln.objective < sln->objective)
        copy_solution(&best_sln, sln);
}
struct solution_struct* greedy_heuristic(struct problem_struct* prob)
{
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
            init_sln->objective += item_i->p;
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


struct solution_struct* find_swarm(struct solution_struct* sln)
{
    struct solution_struct* new_sln = malloc(sizeof(struct solution_struct));
    new_sln->cap_left = malloc(sizeof(int)*sln->prob->dim);
    new_sln->x = malloc(sizeof(int)*sln->prob->n);
    copy_solution(new_sln, sln);

    for(int i=0; i<new_sln->prob->n;i++){
        double avg_size=0;
        struct item_struct* item_i = &new_sln->prob->items[i];
        for(int d=0; d< new_sln->prob->dim; d++){
            avg_size += (double)item_i->size[d]/new_sln->prob->capacities[d];
        }
        item_i->ratio = item_i->p/avg_size;
    }
    qsort(new_sln->prob->items, new_sln->prob->n, sizeof(struct item_struct), cmpfunc1);

    // remove smallest one
    for(int i=(new_sln->prob->n)-1; i>=0;i--){
        if(new_sln->x[new_sln->prob->items[i].index] == 1) {
            new_sln->x[new_sln->prob->items[i].index] = 0;
        }
    }

    qsort(new_sln->prob->items, new_sln->prob->n, sizeof(struct item_struct), cmpfunc2);
    new_sln->objective=0;
    for(int m=0; m<new_sln->prob->n; m++) {
        new_sln->objective += (float)new_sln->x[m] * (float)new_sln->prob->items[m].p;
    }
    return new_sln;
}





void best_descent_swarm(struct solution_struct* sln) {//using pair-swaps
    for (int n = 0; n < SWARM_SIZE; n++) {
        int item1, item2;
        int best_delta = -1000, b_item1 = -1, b_item2 = -1; //store the best move
//        copy_solution(new_sln, &sln[i]);
        for (int i = 0; i < sln[n].prob->n; i++) {
            if (sln[n].x[i] > 0) {
                item1 = i;
                for (int j = 0; j < sln[n].prob->n; j++) {
                    item2 = j;
                    int delta = sln[n].prob->items[item2].p -
                                sln[n].prob->items[item1].p;
                    if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                        b_item1 = item1;
                        b_item2 = item2;
                        best_delta = delta;
                    }
                }//endfor
            }//endif
        }//endfor

        if (best_delta > 0) {
            sln[n].x[b_item1] = 0;
            sln[n].x[b_item2] = 1; //swap
            sln[n].objective = sln[n].objective + (float) best_delta;  //delta evaluation
            //print_sol(new_sln, "Best Descent Solution");

        }
        feasibility_repair(&sln[n]);
    }
    for(int i=0; i<SWARM_SIZE; i++) {
        // update personal best
        if(sln[i].objective>sln[i].personal_best->objective) {
            sln[i].personal_best=&sln[i];
        }
//        check_feasibility(&sln[i]);
//        printf("%d ", sln[i].feasibility);

        //update global best
        if(sln[i].objective>best_sln.objective) {
            update_best_solution(&sln[i]);
//            best_sln=sln[i];
        }
    }
//    check_feasibility(&best_sln);
//    printf("%d ", best_sln.feasibility);


}


void particle_random_swap(struct solution_struct* pop) {
    int swap_index;
    swap_index = rand_int(0, pop->prob->n-1);
    if(pop->x[swap_index]==1) {
        pop->x[swap_index]=0;
    } else {
        pop->x[swap_index]=1;
    }
    feasibility_repair(pop);
}
void particle_random12_swap(struct solution_struct* pop) {
    int swap_index;
    int other2[2];
    swap_index = rand_int(0, pop->prob->n-1);
    if(pop->x[swap_index]==1) {
        for (int i = 0; i < 2; i++) {
            other2[i] = rand_int(0, pop->prob->n-1);
            if(pop->x[other2[i]] == 0) {
                pop->x[other2[i]] = 1;
            } else {
                i--;
            }
        }

    }
    feasibility_repair(pop);
}
void particle_best_descent_11(struct solution_struct* sln) {
    //1-1
    int item1, item2;
    int best_delta = -1000, b_item1 = -1, b_item2 = -1; //store the best move

    int ones[sln->prob->n], zeros[sln->prob->n];
    int ones_index=0;
    int zeros_index=0;
    for (int i = 0; i < sln->prob->n; i++) {
        if(sln->x[i] == 1) {
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    for(int i=0; i<ones_index; i++) {
        item1 = ones[i];
        for (int j = 0; j < zeros_index; j++) {
            item2 = zeros[j];
            int delta = sln->prob->items[item2].p -
                        sln->prob->items[item1].p;
            if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                b_item1 = item1;
                b_item2 = item2;
                best_delta = delta;
            }
        }
    }
    if (best_delta > 0) {
        sln->x[b_item1] = 0;
        sln->x[b_item2] = 1; //swap
        sln->objective = sln->objective + (float) best_delta;  //delta evaluation
    }
    feasibility_repair(sln);
}
void particle_best_descent_12(struct solution_struct* sln) {
    //1-2
    int item1, item2, item3;
    int best_delta = -1000, b_item1 = -1, b_item2 = -1, b_item3 = -1; //store the best move

    int ones[sln->prob->n], zeros[sln->prob->n];
    int ones_index=0;
    int zeros_index=0;
    for (int i = 0; i < sln->prob->n; i++) {
        if(sln->x[i] == 1) {
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    for(int i=0; i<ones_index; i++) {
        item1 = ones[i];
        for(int j=0; j<zeros_index; j++) {
            item2 = rand_int(0,zeros_index-1);
            item3 = rand_int(0,zeros_index-1);
            if(item2!=item3) {
                int delta = sln->prob->items[item2].p + sln->prob->items[item3].p -
                            sln->prob->items[item1].p;
                if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                    b_item1 = item1;
                    b_item2 = item2;
                    b_item3 = item3;
                    best_delta = delta;
                }
            }
        }

    }
    if (best_delta > 0) {
        sln->x[b_item1] = 0;
        sln->x[b_item3] = 1;
        sln->x[b_item2] = 1; //swap
        sln->objective = sln->objective + (float) best_delta;  //delta evaluation
    }
    feasibility_repair(sln);

}
void particle_best_descent_13(struct solution_struct* sln) {
    //1-3
    int item1, item2, item3, item4;
    int best_delta = -1000, b_item1 = -1, b_item2 = -1, b_item3 = -1, b_item4 = -1; //store the best move

    int ones[sln->prob->n], zeros[sln->prob->n];
    int ones_index=0;
    int zeros_index=0;
    for (int i = 0; i < sln->prob->n; i++) {
        if(sln->x[i] == 1) {
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    for(int i=0; i<ones_index; i++) {
        item1 = ones[i];
        for(int j=0; j<zeros_index; j++) {
            item2 = rand_int(0,zeros_index-1);
            item3 = rand_int(0,zeros_index-1);
            item4 = rand_int(0,zeros_index-1);

            if(item2!=item3!=item4) {
                int delta = sln->prob->items[item2].p + sln->prob->items[item3].p + sln->prob->items[item4].p -
                            sln->prob->items[item1].p;
                if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                    b_item1 = item1;
                    b_item2 = item2;
                    b_item3 = item3;
                    b_item4 = item4;
                    best_delta = delta;
                }
            }
        }

    }
    if (best_delta > 0) {
        sln->x[b_item1] = 0;
        sln->x[b_item3] = 1;
        sln->x[b_item4] = 1;
        sln->x[b_item2] = 1; //swap
        sln->objective = sln->objective + (float) best_delta;  //delta evaluation
    }
    feasibility_repair(sln);

}

void particle_best_descent_21(struct solution_struct* sln) {
    //2-1
    int item1, item2, item3;
    int best_delta = -1000, b_item1 = -1, b_item2 = -1, b_item3 = -1; //store the best move

    int ones[sln->prob->n], zeros[sln->prob->n];
    int ones_index=0;
    int zeros_index=0;
    for (int i = 0; i < sln->prob->n; i++) {
        if(sln->x[i] == 1) {
            ones[ones_index] = i;
            ones_index++;
        } else {
            zeros[zeros_index] = i;
            zeros_index++;
        }
    }

    for(int i=0; i<ones_index; i++) {
        item1 = rand_int(0,ones_index-1);
        item2 = rand_int(0,ones_index-1);
        if(item1!=item3) {
            for(int j=0; j<zeros_index; j++) {
                item3 = j;
                int delta = sln->prob->items[item3].p - sln->prob->items[item2].p -
                            sln->prob->items[item1].p;
                if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                    b_item1 = item1;
                    b_item2 = item2;
                    b_item3 = item3;
                    best_delta = delta;
                }
            }
        }
    }
    if (best_delta > 0) {
        sln->x[b_item1] = 1;
        sln->x[b_item3] = 0;
        sln->x[b_item2] = 0; //swap
        sln->objective = sln->objective + (float) best_delta;  //delta evaluation
    }
    feasibility_repair(sln);
}
void random_swap(struct solution_struct* sln) {
    int swap_index;
    swap_index = rand_int(0, sln->prob->n-1);
    if(sln->x[swap_index]==1) {
        sln->x[swap_index]=0;
    } else {
        sln->x[swap_index]=1;
    }
    feasibility_repair(sln);
}
void best_descent_pb(struct solution_struct* sln) {
    //1-1
    int item1, item2;
    int best_delta = -1000, b_item1 = -1, b_item2 = -1; //store the best move
//        copy_solution(new_sln, &sln[i]);
    for (int i = 0; i < sln->prob->n; i++) {
        if (sln->x[i] > 0) {
            item1 = i;
//            item1 = rand_int(0, sln->prob->n-1);
            for (int j = 0; j < sln->prob->n; j++) {
//                item2 = j;
                item2 = rand_int(0, sln->prob->n-1);
                if(sln->x[item2]==0) {
                    int delta = sln->prob->items[item2].p -
                                sln->prob->items[item1].p;
                    if (delta > 0 && delta > best_delta) {   // 如果delta>0，说明新的利润>旧的，取新值
                        b_item1 = item1;
                        b_item2 = item2;
                        best_delta = delta;
                    }
                }
            }//endfor
        }//endif
    }//endfor

    if (best_delta > 0) {
        sln->x[b_item1] = 0;
        sln->x[b_item2] = 1; //swap
        sln->objective = sln->objective + (float) best_delta;  //delta evaluation
        //print_sol(new_sln, "Best Descent Solution");

    }
    feasibility_repair(sln);
}

struct solution_struct* neighbor_select(int i, struct solution_struct* pop) {
//    if(i==1) {
//        particle_random_swap(pop);
//    } else if (i==2){
//        particle_best_descent_11(pop);
//    } else if (i==3){
//        particle_best_descent_12(pop);
//    } else if (i==4){
//        particle_best_descent_21(pop);
//    }
    if(i==1) {
        particle_best_descent_11(pop);
    } else if (i==2){
        particle_best_descent_12(pop);
    } else if (i==3){
        particle_best_descent_21(pop);
    }
    return pop;
}


void particle_VNS(struct solution_struct* sln) {

    int nb_index = 0; //neighbourhood index
    STOP_TIME=clock();
    double time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
    struct solution_struct* curt_sln = sln;

    while(time_spent < MAX_TIME && nb_index<2){
        struct solution_struct* neighbors = neighbor_select(nb_index+1, curt_sln); //best solution in neighbourhood nb_indx
        feasibility_repair(neighbors);
        check_feasibility(neighbors);
        if(neighbors->feasibility<0) {
            printf("after vns\n");
        }
        if(neighbors->objective > curt_sln->objective){
            copy_solution(curt_sln, neighbors);
            nb_index=1;
        }
        else {
            nb_index++;
        }
    }
    sln = curt_sln;
    feasibility_repair(sln);
}


struct solution_struct* update_global_best(struct solution_struct* swarm){
    for(int i=0; i<SWARM_SIZE; i++) {
        struct solution_struct* new_sln = malloc(sizeof(struct solution_struct));
        new_sln->cap_left = malloc(sizeof(int)*swarm[i].prob->dim);
        new_sln->x = malloc(sizeof(int)*swarm[i].prob->n);
//        new_sln->personal_best = malloc(sizeof(struct solution_struct));

        copy_solution(new_sln, &swarm[i]);
        particle_best_descent_11(new_sln);
//        particle_VNS(new_sln);
        if(new_sln->objective>best_sln.objective) {
            feasibility_repair(new_sln);
            copy_solution(&best_sln, new_sln);
        }
    }
}

void update(struct solution_struct* swarm){
    for(int i=0; i<SWARM_SIZE; i++) {
        swarm[i].objective = 0;   // before update each particle, clear its objective value
        for(int j=0; j<swarm->prob->n; j++) {
            float pp = rand_01();
            float pg = rand_01();

            double w=1;
            float Cp=2;
            float Cg=2;
//            double w=0.7298;
//            double Cp=1.49618;
//            double Cg=1.49618;
            float V_MAX=5;
//            float V_MIN=0;

            double K=0.729;

            // update velocity
            swarm[i].v[j] = w*swarm[i].v[j] + pp*Cp*(float)(swarm[i].personal_best->x[j]-swarm[i].x[j]) + pg*Cg*(float)(best_sln.x[j]-swarm[i].x[j]);
//            swarm[i].v[j] = K*(swarm[i].v[j] + pp*Cp*(swarm[i].personal_best->x[j]-swarm[i].x[j]) + pg*Cg*(best_sln.x[j]-swarm[i].x[j]));


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
//        if(swarm[i].objective>best_sln.objective) {
//            best_sln=swarm[i];
//        }
    }
    update_global_best(swarm);

//    printf("pre best: %f\n", best_sln.objective);
//    particle_best_descent_11(&best_sln);
//    printf("after best: %f\n", best_sln.objective);
//    if((next_best_sln.objective-best_sln.objective)<1000) {
//
//    } else {
//        best_sln = next_best_sln;
//    }
}



/**
 * initialize particle swarm, give initial position, velocity, personal best and global best
 * ignore the feasibility of each particle
 *
 * And finally repair them
 */
int initialize_particle_swarm(struct problem_struct* prob, struct solution_struct* sln) {
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
        // find the neighborhood of current solution, and assign the biggest neighbor as initial global best
        particle_best_descent_11(curt_sln);
        update_best_solution(curt_sln);

        // initialize position and velocity randomly for each particle
        for(int j=0; j<prob->n; j++) {
            sln[i].x[j] = rand()%2;
            sln[i].objective += (float)sln[i].x[j] * (float)sln[i].prob->items[j].p;
            sln[i].v[j] = rand_01();
//            sln[i].v[j] = -1+2*rand_01();
//            sln[i].v[j] = (float)rand_int(0, 5);
        }

//        fitness_calculate(&sln[i]);
        // repair feasibility of each particle
        feasibility_repair(&sln[i]);
        // check feasibility
        check_feasibility(&sln[i]);
        if(sln[i].feasibility<0) {
            printf("infeasible particle %d in initialize_particle_swarm\n", i);
        }
    }
}

void VNS(struct solution_struct* swarm) {
    for(int i = 0; i < SWARM_SIZE; i++){
        int nb_indx = 0; //neighbourhood index
        STOP_TIME=clock();
        double time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
        struct solution_struct* curt_sln = &swarm[i];

        while(time_spent < MAX_TIME && nb_indx<2){
            struct solution_struct* neighbor = neighbor_select(nb_indx+1, curt_sln); //best solution in neighbourhood nb_indx
            // feasibility_repair(neighbor);
            if(neighbor->objective > curt_sln->objective){
                copy_solution(curt_sln, neighbor);
                nb_indx=1;
            }
            else nb_indx++;
            // free_solution(neighb_s);free(neighb_s);
        }

        swarm[i] = *curt_sln;
        if(swarm[i].objective>swarm[i].personal_best->objective) {
            copy_solution(swarm[i].personal_best, &swarm[i]);
        }
    }
    update_global_best(swarm);

//    for(int i=0; i<SWARM_SIZE; i++) {
//
//
//
//        // update personal best
////        if(swarm[i].objective>swarm[i].personal_best->objective) {
////            swarm[i].personal_best=&swarm[i];
////        }
////        check_feasibility(&sln[i]);
////        printf("%d ", sln[i].feasibility);
//
//        //update global best
//        if(swarm[i].objective>best_sln.objective) {
//            best_sln=swarm[i];
//        }
//
//        check_feasibility(&best_sln);
//        if(best_sln.feasibility<0) {
//            printf("in VNS\n");
//        }
//    }
}


int PSO(struct problem_struct* prob) {
    START_TIME = clock();
    double time_spent=0;
    int iter =0;

    // create a particle swarm with size SWARM_SIZE
    struct solution_struct particle_swarm[SWARM_SIZE];
    // initialize the particle swarm
    initialize_particle_swarm(prob, particle_swarm);

//    while(iter<138 && time_spent < MAX_TIME) {
    while(time_spent < MAX_TIME) {
        update(particle_swarm);
        VNS(particle_swarm);
//        particle_best_descent_11(&best_sln);




//        int rand_index1 = rand_int(0, (int)(SWARM_SIZE/2));
//        random_swap(&particle_swarm[rand_index1]);
//        int rand_index2 = rand_int((int)(SWARM_SIZE/2)+1, SWARM_SIZE-1);
//        random_swap(&particle_swarm[rand_index2]);



//        int rand_index = rand_int(0, SWARM_SIZE-1);
//        random_swap(&particle_swarm[rand_index]);



//        particle_best_descent_11(&particle_swarm[rand_index]);
//        update_global_best(particle_swarm);
//        VNS(particle_swarm);
        iter++;
        STOP_TIME=clock();
        time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
        //printf("best: %f  worst: %f   iter: %d  time: %f\n", parent_pop[0].objective, parent_pop[POP_SIZE-1].objective, iter, time_spent);
    }
//    printf("feasibility: %d\n", best_sln.feasibility);

//     update_best_solution(particle_swarm);


    //printf("optimal: %d\n", parent_pop->prob->optimal);
//    printf("gap: %f  worst: %f   iter: %d  time: %f\n", (parent_pop->prob->optimal-parent_pop[0].objective)/parent_pop->prob->optimal, (parent_pop->prob->optimal-parent_pop[POP_SIZE-1].objective)/parent_pop->prob->optimal, iter, time_spent);
//    printf("gap: %f  worst: %f   iter: %d  time: %f\n", (particle_swarm->prob->optimal-particle_swarm[0].objective)/particle_swarm->prob->optimal, (particle_swarm->prob->optimal-particle_swarm[SWARM_SIZE-1].objective)/particle_swarm->prob->optimal, iter, time_spent);



//    printf("%d\n", best_sln.feasibility);
//    for(int m=0; m<SWARM_SIZE; m++) {
//        check_feasibility(&particle_swarm[m]);
//        if(particle_swarm[m].feasibility<0) {
//            printf("%d\n", particle_swarm[m].feasibility);
//        }
//    }
//    feasibility_repair(&best_sln);
    check_feasibility(&best_sln);
    if(best_sln.feasibility<0) {
        printf("global best is not feasible, before print\n");
    }
    printf("best: %f iter: %d time: %f\n", best_sln.objective, iter, time_spent);
//    printf("%f\n", best_sln.objective);

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
        //printf("data_file= %s, output_file= %s, sln_file=%s, max_time=%d", data_file, out_file, solution_file, MAX_TIME);
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
//                printf("something wrong before!\n");

            }
//           s feasibility_repair(&best_sln);
//            printf("best: %f  problem: %d\n", best_sln.objective, k);
//             output_solution(&best_sln,out_file);
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