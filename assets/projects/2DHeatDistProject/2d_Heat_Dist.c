/* CS_441_Assignment#3.c
 * Description: Program that pratices the utilizes MPI to Solve a Partial Differential Equation
 * Programmer: Adrian Beehner
 * Date: 4/1/17

Problem: 
* Write a distributed program, using MPI, tha will solve the PDE described above. since the calculation over
* the array is quite parallel, break up the array into "stripes" and have one processor calculate each stripe.
* The elements on the boundaries between each stripe need the new values that were caculated in the other stripe
* in order to complete their caclulations. Thus, each stripe will need to corrdinate with its adjacent stripes.
* This will rrequire a communication between the stripes.*/


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>						// Form implementing Jacobi method
#include <mpi.h>						// For using MPI

double TOLERANCE = .05;					// Varable for tolerance

// Jacobi iteration
double update(float *hp, int p, int P, int n, int m, double ch, int itr);

int main(int argc, char *argv[])
{
	int p, P;
	int n, m;
	int i, j;
	float *hp;
	double t0, tf;
	int iter = 0;
	double change;
  
	//**********************
	// Read in the argument
	//**********************
	if ( argc < 2 || argc > 2 )			// If 1 argument is NOT passed
	{
										// perror(...) for printing error message

		perror("Two arguments are exptected, terminating program.\n\n");

		exit(1);						// Terminate with error

	}
	// Otherwise 1 argument are passed
	// Since *argv is in char, need to convert our argument from ASCII to in
	int arg[1];							// Initialize array to hold int values of argv

	for (int k = 1; k < argc; k++)		// k < # of arguments, k = 1 to Skip program name
		{								//			when looking through argument array
			arg[k-1] = atoi(argv[k]);  	// Convert the argument in argv[k] from string to
		}								// int with new array arg[k] (we have to do k-1
										//		because we initalized int k = 1, but arrays
										//		MUST start at 0! so arr[0],arr[1],etc...
	
	// Assign the argument meant to express the dimension into a variable
	n = arg[0];							// Define n for mxn
	m = arg[0] - 1; 					// Define m (must do M-1, even though it will still act as M)
										// 		ex: 9 - 1, will be value 8, but prigram will use it as M = 9

	// Start up MPI
	MPI_Init(&argc, &argv);
	
	// Assign Process rank to p
	MPI_Comm_rank(MPI_COMM_WORLD, &p);
	
	// Assign number of processes to P
	MPI_Comm_size(MPI_COMM_WORLD, &P);
	
	// Allocate memory for grid, called hp
	hp = malloc(sizeof(float)*n*m);


	// apply  boundaries */
	for (i = 0; i < m; i++)
	{
		for (j = 0; j < n; j++)
		{
			// top row
			if (i == 0)
				hp[n*i+j] = 100;
			// bottom row
			if (i == (m-1))
				hp[n*i+j] = 0;
			// left column
			if (j == 0)
				hp[n*i+j] = 100;
			// right column
			if (j == (n-1))
				hp[n*i+j] = 100;
		}
	}
	
	if (p == 0)
	{	
		printf("\n\nCS 441 Assignment 3: MPI PDE --- Adrian Beehner\n\n");
		printf("If the nxn has n>20, the grid cells will only be shown per 8 cells\n\n");
		
		printf("\nStarting Grid:\n");
		printf("--------------\n\n");
		
		if (n < 20)
		{
			for (i = 0; i < m; i++)
			{
				for (j = 0; j < n; j++)
				{
					// Format so it will pad output in a field 10 characters long
					printf("%-10lf ", hp[n*i+j]);
				}
				
				printf("\n");
			}
			
			// Print out Bottomw bounadry, as cells will skip it
			 for (j = 0; j < n; j++)
			 {
				 i = m; 
				 
				 if (i == m)
					printf("%-10lf ", hp[n*i+j]);
			 }
		}
		else
		{
			for (i = 0; i < m; i = i + 9)
			{
				for (j = 0; j < n; j = j + 9)
				{
					// Format so it will pad output in a field 10 characters long
					printf("%-10lf ", hp[n*i+j]);
				}
				printf("\n");
			}
			
			// Print out Bottomw bounadry, as cells will skip it
			 for (j = 0; j < n; j = j + 9)
			 {
				 i = m; 
				 
				 if (i == m)
					printf("%-10lf ", hp[n*i+j]);
			 }
			
			// Print bottom boundary, as it is not shown when showing only 8 cells
		}
	}	
	
	MPI_Barrier(MPI_COMM_WORLD);
	
	// Start timer
	t0 = MPI_Wtime();

	//***********************************************
	// Run iterations until tolerance is acheived
	//***********************************************
	while(1)
	{
		change = 0.0;
		
		change = update(hp, p, P, n, m, change, iter);
		
		iter++;
		
		// Make sure we end with Master Process
		if (p == 0)
		{
			// End only if we got desired tolerance
			if(TOLERANCE >= change && change != 0.0)
			{
				// End Timer
				tf = MPI_Wtime();
				
				
				
				// Print out Info
				printf("\n\n");
				printf("m: %d X n: %d\n", (m+1), n);
				printf("Change: %lf, Tolerance: %lf, iterations: %d\n", change, TOLERANCE, iter);
				printf("Number of Processes = %d\nIterations = %d\nTime = %gs\nTolerance = %g\n", P, (iter-1), tf-t0, (float)TOLERANCE);
				
				//done = 1;
				
				// Print out results
				printf("\nResults:\n");
				printf("--------\n\n");
		
				if (n < 20)
				{
					for (i = 0; i < m; i++)
					{
						for (j = 0; j < n; j++)
						{
							// Format so it will pad output in a field 10 characters long
							printf("%-10lf ", hp[n*i+j]);
						}
						
						printf("\n");
					}
					
					// Print out Bottomw bounadry, as cells will skip it
					 for (j = 0; j < n; j++)
					 {
						 i = m; 
						 
						 if (i == m)
							printf("%-10lf ", hp[n*i+j]);
					 }
				}
				else
				{
					for (i = 0; i < m; i = i + 9)
					{
						for (j = 0; j < n; j = j + 9)
						{
							// Format so it will pad output in a field 10 characters long
							printf("%-10lf ", hp[n*i+j]);
						}
						printf("\n");
					}
					
					// Print out Bottomw bounadry, as cells will skip it
					 for (j = 0; j < n; j = j + 9)
					 {
						 i = m; 
						 
						 if (i == m)
							printf("%-10lf ", hp[n*i+j]);
					 }
					
					// Print bottom boundary, as it is not shown when showing only 8 cells
				}
				
				printf("\n");
				return 0;
			
			}
		}
	}
	
	// Finish running the program
	MPI_Barrier(MPI_COMM_WORLD);
	
	free(hp);
	
	MPI_Finalize();
	
	return 0;
}

/************************************************************
 * 						  update							*
 * 	function that implemets the Jacobi iteration wit		*
 * 	overapping computation/communication. The grid accepts  *
 *  a 2d grid, the processor rank, the number of processors	*
 *  the sze of a nxn grid, and the current tolerancethe 	*
 * 	The communication is utilized by the MPI_Sendrev 		*
 * 	function.												*
 * **********************************************************/
double update(float *hp, int p, int P, int n, int m, double ch, int itr)
{
	int i, j;
	//int stop = 0;
	static float *hnew = NULL;	/// static keeps value accross calls 
	static int hnew_size = 0;
	double change = ch;
	MPI_Status status;

	//allocate only when needed.
	//Note 1: static *hnew keeps value across calls so can reuse its space!
	//Note 2: that this is not MT-safe!

	if (hnew_size != n*m)
	{
		if (hnew)
		  free(hnew);
		hnew = malloc(sizeof(float)*n*m);
		hnew_size = n*m;
	}
	
	// If Process is not first process and not the last process
	if (p > 0 && p < P-1)
	{
		MPI_Sendrecv(&hp[n*(m-2)], n, MPI_FLOAT, p+1, 0, &hp[n*0], n, MPI_FLOAT, p-1, 0, MPI_COMM_WORLD, &status);
		MPI_Sendrecv(&hp[n*1], n, MPI_FLOAT, p-1, 1, &hp[n*(m-1)], n, MPI_FLOAT, p+1, 1, MPI_COMM_WORLD, &status);
	}
	
	// If porcess is the first process (2nd argument is bounds checking)
	else if (p == 0 && p < P-1)
	{
		MPI_Sendrecv(&hp[n*(m-2)], n, MPI_FLOAT, p+1, 0, &hp[n*(m-1)], n, MPI_FLOAT, p+1, 1, MPI_COMM_WORLD, &status);
	}
	
	// If the process is the last process (2nd argument is bounds checking)
	else if (p > 0 && p == P-1)
	{
		MPI_Sendrecv(&hp[n*1], n, MPI_FLOAT, p-1, 1, &hp[n*0], n, MPI_FLOAT, p-1, 0, MPI_COMM_WORLD, &status);
	}
	
	//Start of computation 
	for (i = 1; i < m-1; i++)
		for (j = 1; j < n-1; j++)
			hnew[n*i+j] = 0.25f*(hp[n*(i-1)+j]+hp[n*(i+1)+j]+hp[n*i+j-1]+hp[n*i+j+1]);
	// End of computation
	
	// Copy newly change grid (hnew)
	for (i = 1; i < m-1; i++)
	{
		for (j = 1; j < n-1; j++)
		{
			if ( change < fabs ( hp[n*i+j] - hnew[n*i+j] ) );
			{
				change = fabs ( hp[n*i+j] - hnew[n*i+j] );
			}
			hp[n*i+j] = hnew[n*i+j];
		}
	}
	
	char processor_name[MPI_MAX_PROCESSOR_NAME];
    int name_len;
    MPI_Get_processor_name(processor_name, &name_len);
	
	
	// Show around the halfway point fot the iterations  (64 is around the average)
	if (itr == 32)
	{
		printf("\n\nGrid for process %d from processor %s at iteration %d\n", p, processor_name, itr);
		if (n < 20)
		{
			for (i = 0; i < m; i++)
			{
				for (j = 0; j < n; j++)
				{
					// Format so it will pad output in a field 10 characters long
					printf("%-10lf ", hp[n*i+j]);
				}
				
				printf("\n");
			}
			
			// Print out Bottomw bounadry, as cells will skip it
			 for (j = 0; j < n; j++)
			 {
				 i = m; 
				 
				 if (i == m)
					printf("%-10lf ", hp[n*i+j]);
			 }
		}
		else
		{
			for (i = 0; i < m; i = i + 9)
			{
				for (j = 0; j < n; j = j + 9)
				{
					// Format so it will pad output in a field 10 characters long
					printf("%-10lf ", hp[n*i+j]);
				}
				printf("\n");
			}
			
			// Print out Bottomw bounadry, as cells will skip it
			 for (j = 0; j < n; j = j + 9)
			 {
				 i = m; 
				 
				 if (i == m)
					printf("%-10lf ", hp[n*i+j]);
			 }
			
			// Print bottom boundary, as it is not shown when showing only 8 cells
		}
	}
	
	return change;
}
