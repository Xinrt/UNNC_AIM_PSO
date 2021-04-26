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

/* parameters for PSO algorithms */
int SWARM_SIZE=30;    // [20, 40]
int MAX_NUM_OF_ITER = 10000;
// range of particle's velocity and position
const float V_MAX=10;


// coefficient
//double Cp = 0.5;   // <=2
//double Cg = 2.5;   // <=2
//double w = 0.8;  // [0.4, 0.9]

double Cp_i = 2.5;   // <=2
double Cp_f = 0.5;
double Cg_f = 2.5;   // <=2
double Cg_i = 0.5;
double w_max = 0.9;  // [0.4, 0.9]
double w_min = 0.4;

struct solution_struct best_sln;  //global best solution
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
//    for(int m=0; m<pop->prob->n; m++) {
//        printf("%d", pop->prob->items[m].index);
////        printf("+%f ", pop->prob->items->ratio);
//    }
//    printf("\n");
//    printf("------------------------------------------\n");


    check_feasibility(pop);

    if(pop->feasibility<0) {
//        printf("aaaaaaaa\n");
        for(int i=(pop->prob->n)-1; i>=0;i--){
//            printf("f1: %d\n", pop->feasibility);
            if(pop->x[pop->prob->items[i].index] == 1) {
                pop->x[pop->prob->items[i].index] = 0;
                check_feasibility(pop);
//                printf("f2: %d\n", pop->feasibility);
                if(pop->feasibility>0) {
                    break;
                }
            }
        }
    } else {
        for(int i=0; i<pop->prob->n;i++){
            if(pop->x[pop->prob->items[i].index] == 0) {
                pop->x[pop->prob->items[i].index] = 1;
                check_feasibility(pop);
                if(pop->feasibility<0) {
                    pop->x[pop->prob->items[i].index] = 0;
                    check_feasibility(pop);
                    break;
                }
            }
        }
    }
    qsort(pop->prob->items, pop->prob->n, sizeof(struct item_struct), cmpfunc2);
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

void random_select(struct solution_struct* swarm) {
    for(int i=0; i<SWARM_SIZE; i++) {
//
//        int swap_pair[1];
//        for(int j=0; j<1; j++) {
//            swap_pair[j] = rand_int(0, swarm[i].prob->n-1);
//            if(swarm[i].x[swap_pair[j]]==0) {
//                swarm[i].x[swap_pair[j]]=1;
//            } else {
//                j--;
//            }
//        }

//        int swap_index;
//        swap_index = rand_int(0, swarm[i].prob->n-1);
//        swarm[i].x[swap_index]=1;

        int swap_index;
        swap_index = rand_int(0, swarm[i].prob->n-1);
        if(swarm[i].x[swap_index]==1) {
            swarm[i].x[swap_index]=0;
        } else {
            swarm[i].x[swap_index]=1;
        }
        feasibility_repair(&swarm[i]);
    }
}

void best_descent(struct solution_struct* sln)
{//using pair-swaps
    for(int n=0; n<SWARM_SIZE; n++) {
        int item1, item2;
        int best_delta = -1000, b_item1=-1, b_item2=-1; //store the best move
//        copy_solution(new_sln, &sln[i]);
        for(int i=0; i<sln[n].prob->n; i++)
        {
            if(sln[n].x[i]>0){
                item1 =i;
                for(int j=0; j<sln[n].prob->n; j++){
                    item2=j;
                    int delta =sln[n].prob->items[item2].p -
                            sln[n].prob->items[item1].p;
                    if(delta >0 && delta>best_delta){   // 如果delta>0，说明新的利润>旧的，取新值
                        b_item1 = item1; b_item2 = item2;
                        best_delta= delta;
                    }
                }//endfor
            }//endif
        }//endfor

        if(best_delta>0){
            sln[n].x[b_item1]= 0;
            sln[n].x[b_item2]= 1; //swap
            sln[n].objective=sln[n].objective + (float)best_delta;  //delta evaluation
            //print_sol(new_sln, "Best Descent Solution");
        }
    }
}

struct solution_struct* new_best_descent(struct solution_struct* sln)
{//using pair-swaps
//    for(int n=0; n<SWARM_SIZE; n++) {
        int item1, item2;
        int best_delta = -1000, b_item1=-1, b_item2=-1; //store the best move
        struct solution_struct* new_sln = malloc(sizeof(struct solution_struct));
        new_sln->objective = 0;
        new_sln->x = malloc(sizeof(int)*sln->prob->n);
        new_sln->cap_left = malloc(sizeof(int)*sln->prob->dim);
        new_sln->v = malloc(sizeof(float )*sln->prob->n);
        new_sln->personal_best = malloc(sizeof(struct solution_struct));




    copy_solution(new_sln, sln);
        for(int i=0; i<sln->prob->n; i++)
        {
            if(new_sln->x[i]>0){
                item1 =i;
                for(int j=0; j<sln->prob->n; j++){
                    item2=j;
                    int delta =new_sln->prob->items[item2].p -
                            new_sln->prob->items[item1].p;
                    if(delta >0 && delta>best_delta){   // 如果delta>0，说明新的利润>旧的，取新值
                        b_item1 = item1; b_item2 = item2;
                        best_delta= delta;
                    }
                }//endfor
            }//endif
        }//endfor

        if(best_delta>0){
            new_sln->x[b_item1]= 0;
            new_sln->x[b_item2]= 1; //swap
            new_sln->objective=sln->objective + (float)best_delta;  //delta evaluation
            //print_sol(new_sln, "Best Descent Solution");
            return new_sln;
        } else {
            return sln;
        }
//    }
}




/**
 * initialize particle swarm, give initial position, velocity, personal best and global best
 * ignore the feasibility of each particle
 */
int initialize_particle_swarm(struct problem_struct* prob, struct solution_struct* sln) {
    for(int i=0; i<SWARM_SIZE; i++) {
        // malloc spaces
        sln[i].objective = 0;
        sln[i].prob = prob;
        sln[i].x = malloc(sizeof(int)*prob->n);
        sln[i].cap_left = malloc(sizeof(int)*prob->dim);
        sln[i].v = malloc(sizeof(float )*prob->n);
//        sln[i].personal_best = malloc(sizeof(struct solution_struct)*prob->n);
        sln[i].personal_best = malloc(sizeof(struct solution_struct));
        sln[i].personal_best->cap_left = malloc(sizeof(int)*sln[i].prob->dim);
        sln[i].personal_best->x = malloc(sizeof(int)*sln[i].prob->n);

        // initially assign particle itself as personal best
        copy_solution(sln[i].personal_best, &sln[i]);
//        sln[i].personal_best=&sln[i];

        // initialize value of particle's position, velocity and calculate the objective value
        // ignore the feasibility of the particle
        for(int j=0; j<prob->n; j++) {
            sln[i].x[j] = rand()%2;
//            update_best_solution(&sln[i]);
            sln[i].objective += (float)sln[i].x[j] * (float)sln[i].prob->items[j].p;
            sln[i].v[j] = rand_01();
        }
        feasibility_repair(&sln[i]);
    }

    // find the biggest objective and reassign the global best particle
    best_sln=sln[0];    // initially assign first particle as global best
    float max_objective = sln[0].objective;
    for(int i=0; i<SWARM_SIZE; i++) {
        if(sln[i].objective>max_objective) {
            max_objective=sln[i].objective;
            best_sln=sln[i];
        }
    }
//    printf("objective %f\n", best_sln.objective);
}

void update(struct solution_struct* swarm){
    for(int i=0; i<SWARM_SIZE; i++) {
        swarm[i].objective = 0;   // before update each particle, clear its objective value

        for(int j=0; j<swarm->prob->n; j++) {
            float pp = rand_01();
            float pg = rand_01();

            // update coefficient
//            double w = (w_max-w_min)*((MAX_TIME-((double)(clock())/CLOCKS_PER_SEC))/MAX_TIME) + w_min;
//            double Cp = (Cp_f-Cp_i)*((((double)(clock())/CLOCKS_PER_SEC)-1)/MAX_TIME) + Cp_i;
//            double Cg = (Cg_f-Cg_i)*((((double)(clock())/CLOCKS_PER_SEC)-1)/MAX_TIME) + Cg_i;

            double w=0.7298;
            double Cp=1.49618;
            double Cg=1.49618;



            // update velocity
            swarm[i].v[j] = w*swarm[i].v[j] + pp*Cp*(swarm[i].personal_best->x[j]-swarm[i].x[j]) + pg*Cg*(best_sln.x[j]-swarm[i].x[j]);

            if(swarm[i].v[j]>V_MAX) {
                swarm[i].v[j]=V_MAX;
            }
            if(swarm[i].v[j]<-V_MAX) {
                swarm[i].v[j]=-V_MAX;
            }

            // sigmoid type function, move to new position
            float random_number = rand_01();
            if(random_number>=1/(1+expf(-swarm[i].v[j]))) {
                swarm[i].x[j]=0;
            } else {
                swarm[i].x[j]=1;
            }
            // recalculate the objective
//            update_objective(&swarm[i]);
            swarm[i].objective += (float)swarm[i].x[j] * (float)swarm[i].prob->items[j].p;
        }
        feasibility_repair(&swarm[i]);
    }

    for(int i=0; i<SWARM_SIZE; i++) {
        // update personal best
        if(swarm[i].objective>swarm[i].personal_best->objective) {
            swarm[i].personal_best=&swarm[i];
        }

        //update global best
        if(swarm[i].objective>best_sln.objective) {
            best_sln=swarm[i];
        }
    }
}

//void newUpdate(struct solution_struct* swarm){
//    for(int i=0; i<SWARM_SIZE; i++) {
//        swarm[i].objective = 0;   // before update each particle, clear its objective value
//
//        for(int j=0; j<swarm->prob->n; j++) {
//            float pp = rand_01();
//            float pg = rand_01();
//
//            // update coefficient
//            double w=0.7298;
//            double Cp=2.8;
//            double Cg=1.8;
//            double K;
//            double phy = Cp+Cg;
//            K = 2 / abs(2-phy-sqrt(phy*phy-4*phy));
//
//            // update velocity
//            swarm[i].v[j] = K*(swarm[i].v[j] + pp*Cp*(swarm[i].personal_best->x[j]-swarm[i].x[j]) + pg*Cg*(best_sln.x[j]-swarm[i].x[j]));
//
////            if(swarm[i].v[j]>V_MAX) {
////                swarm[i].v[j]=V_MAX;
////            }
////            if(swarm[i].v[j]<-V_MAX) {
////                swarm[i].v[j]=-V_MAX;
////            }
//
//            // sigmoid type function, move to new position
//            float random_number = rand_01();
//            if(random_number>=1/(1+exp(-swarm[i].v[j]))) {
//                swarm[i].x[j]=0;
//            } else {
//                swarm[i].x[j]=1;
//            }
//            // recalculate the objective
//            swarm[i].objective += swarm[i].x[j] * swarm[i].prob->items[j].p;
//        }
//    }
//
//    for(int i=0; i<SWARM_SIZE; i++) {
//        // update personal best
//        if(swarm[i].objective>swarm[i].personal_best->objective) {
//            swarm[i].personal_best=&swarm[i];
//        }
//
//        //update global best
//        if(swarm[i].objective>best_sln.objective) {
//            best_sln=swarm[i];
//        }
//    }
//}

//void newUpdate2(struct solution_struct* swarm){
//    for(int i=0; i<SWARM_SIZE; i++) {
//        swarm[i].objective = 0;   // before update each particle, clear its objective value
//
//        for(int j=0; j<swarm->prob->n; j++) {
//            float pp = rand_01();
//            float pg = rand_01();
//
//            // update coefficient
//            double w=0.7298;
//            double Cp=1.49618;
//            double Cg=1.49618;
//            double alpha = rand_01();
//            struct solution_struct* curt_sln = &swarm[i];
//
//            struct solution_struct *neighbor_sln = new_best_descent(curt_sln);
//
//            // update velocity
//            swarm[i].v[j] = w*(swarm[i].v[j] + pp*Cp*alpha*(swarm[i].personal_best->x[j]-swarm[i].x[j]) + (1-alpha)*(neighbor_sln->x[j]-swarm[i].x[j]) + pg*Cg*(best_sln.x[j]-swarm[i].x[j]));
//
////            if(swarm[i].v[j]>V_MAX) {
////                swarm[i].v[j]=V_MAX;
////            }
////            if(swarm[i].v[j]<-V_MAX) {
////                swarm[i].v[j]=-V_MAX;
////            }
//
//            // sigmoid type function, move to new position
//            float random_number = rand_01();
////            printf("%f   ", swarm[i].v[j]);
//            if(random_number>=1/(1+exp(-swarm[i].v[j]))) {
//                swarm[i].x[j]=0;
//            } else {
//                swarm[i].x[j]=1;
//            }
//            // recalculate the objective
//            swarm[i].objective += swarm[i].x[j] * swarm[i].prob->items[j].p;
//        }
//        feasibility_repair(&swarm[i]);
//    }
//
//    for(int i=0; i<SWARM_SIZE; i++) {
//        // update personal best
//        if(swarm[i].objective>swarm[i].personal_best->objective) {
//            swarm[i].personal_best=&swarm[i];
//        }
//
//        //update global best
//        if(swarm[i].objective>best_sln.objective) {
//            best_sln=swarm[i];
//        }
//    }
//}




int PSO(struct problem_struct* prob) {
    START_TIME = clock();
    double time_spent=0;
    int iter =0;

    // create a particle swarm with size SWARM_SIZE
    struct solution_struct particle_swarm[SWARM_SIZE];
    // initialize the particle swarm
    initialize_particle_swarm(prob, particle_swarm);

    while(iter<MAX_NUM_OF_ITER && time_spent < MAX_TIME) {
//        update(particle_swarm);
//        newUpdate2(particle_swarm);
        update(particle_swarm);
        random_select(particle_swarm);
//        best_descent(particle_swarm);
//        SimulatedAnnealing(&best_sln);


//        VNS(particle_swarm);

        iter++;
        STOP_TIME=clock();
        time_spent = (double)(STOP_TIME-START_TIME)/CLOCKS_PER_SEC;
        //printf("best: %f  worst: %f   iter: %d  time: %f\n", parent_pop[0].objective, parent_pop[POP_SIZE-1].objective, iter, time_spent);
    }
//    feasibility_repair(&best_sln);
//    printf("feasibility: %d\n", best_sln.feasibility);

//     update_best_solution(particle_swarm);


    //printf("optimal: %d\n", parent_pop->prob->optimal);
//    printf("gap: %f  worst: %f   iter: %d  time: %f\n", (parent_pop->prob->optimal-parent_pop[0].objective)/parent_pop->prob->optimal, (parent_pop->prob->optimal-parent_pop[POP_SIZE-1].objective)/parent_pop->prob->optimal, iter, time_spent);
//    printf("gap: %f  worst: %f   iter: %d  time: %f\n", (particle_swarm->prob->optimal-particle_swarm[0].objective)/particle_swarm->prob->optimal, (particle_swarm->prob->optimal-particle_swarm[SWARM_SIZE-1].objective)/particle_swarm->prob->optimal, iter, time_spent);


//    printf("best: %f  worst: %f   iter: %d  time: %f\n", best_sln.objective, particle_swarm[SWARM_SIZE-1].objective, iter, time_spent);
    printf("%f\n", best_sln.objective);

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