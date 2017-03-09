#include"opencv2/opencv.hpp"
#include<opencv2/core/core.hpp>
#include"opencv2/imgproc/imgproc.hpp"
#include<opencv2/highgui/highgui.hpp>
#include<queue>
#include<functional>
#include <vector>

using namespace std;

using namespace cv;
Mat img = imread("C:\\Users\\Aryan\\Documents\\Visual Studio 2013\\Projects\\OpenCVProject3\\OpenCVProject3\\pic.jpg", 1);
Mat img1(img.rows, img.cols, CV_8UC3, Scalar(0, 0, 0));
Mat img2(img.rows, img.cols, CV_8UC3, Scalar(0, 0, 0));
struct point { int x, y; };                                      //structure for storing coordinates
struct state { point p; double c; };							//structure used in queue for coordinate and cost
int isValid(int i, int j, Mat img1)										//function checking the validity of the coordinates
{
	if ((i < 0) || (j < 0) || (i >= img1.rows) || (j >= img1.cols))
		return 0;
	else
		return 1;

}
void filter(struct point start, struct point end)                    //function to filter the image by extracting each 
{																	// square grid and converting all black points to white
	int i, j, k, l;													//in each grid.
	for (i = 0; i < img.rows; i += end.x)
	{
		for (j = 0; j < img.cols; j += end.y)
		{
			for (k = i + 3; k < i + end.x - 2; k++)
			{
				for (l = j + 3; l <j + end.y - 2; l++)
				{
					if (isValid(k, l, img))
					{

						if (img.at<Vec3b>(k, l)[0] <= 40 && img.at<Vec3b>(k, l)[1] <= 40 && img.at<Vec3b>(k, l)[2] <= 40)
						{
							img.at<Vec3b>(k, l)[0] = 255;
							img.at<Vec3b>(k, l)[1] = 255;
							img.at<Vec3b>(k, l)[2] = 255;

						}
					}
				}
			}
		}
	}
}

int cost(int k, int l)                                                   //cost function returning cost of each point traversed
{
	if (img.at<Vec3b>(k, l)[0] > 245 && img.at<Vec3b>(k, l)[1] > 245 && img.at<Vec3b>(k, l)[2] > 245)
		return 1000000;
	else if (img.at<Vec3b>(k, l)[0] > 245 && img.at<Vec3b>(k, l)[1] < 10 && img.at<Vec3b>(k, l)[2] < 10)
		return 20;
	else if (img.at<Vec3b>(k, l)[0]<10 && img.at<Vec3b>(k, l)[1] >245 && img.at<Vec3b>(k, l)[2] < 10)
		return 15;
	else if (img.at<Vec3b>(k, l)[0] < 10 && img.at<Vec3b>(k, l)[1] <10 && img.at<Vec3b>(k, l)[2]>245)
		return 12;
	else if (img.at<Vec3b>(k, l)[0] == 128 && img.at<Vec3b>(k, l)[1] == 128 && img.at<Vec3b>(k, l)[2] < 10)
		return 10;
	else if (img.at<Vec3b>(k, l)[0] < 10 && img.at<Vec3b>(k, l)[1] == 128 && img.at<Vec3b>(k, l)[2] == 128)
		return 9;
	else
		return 0;
}
double heuristics(point current, point goal)                                 //heuristics calculating manhattan distance
{
	return abs(goal.x - current.x) + abs(current.y - goal.y);
}

void dijkstra(struct point start, struct point end,int **cost_so_far, double **priority)			//function to traverse each point from start to end and store each pixel's cost
{																									//in the created arrays
	state current;
	double new_cost;
	int k, l;
	queue<state> q;
	q.push({ start,0});																	//creating queue of structure state holding both coordinate and cost
	cost_so_far[start.x][start.y] = 0;													//storing cost of reaching a particular coordinate from start
	priority[start.x][start.y] = 0;															//storint cost alongwith heuristics(manhattan)

	while (!q.empty())
	{
		current = q.front();
		q.pop();
		img1.at<Vec3b>(current.p.x, current.p.y)[1] = 255;
		if (current.p.x == end.x&&current.p.y == end.y)
			break; 
		int c = 0;
		for (k = current.p.x - end.x; k <= current.p.x + end.x; k += end.x)                                         //travesing through vertex of squares
		{																											//checking only the top-bottom-left and right nodes 
			c++;																									//here only rows
			if (c == 1 || c == 3)
			{
				l = current.p.y;
				if (isValid(k, l, img))
				{
					new_cost = current.c + cost(k, l);																//storing cost of reaching a particular coordinate
					if (img1.at<Vec3b>(k, l)[1] != 255 || (new_cost < cost_so_far[k][l]))							//checking if visited OR new cost of raching that 
					{																								//pixel is less than previous cost
						cost_so_far[k][l] = new_cost;
						priority[k][l] = cost_so_far[k][l] + heuristics({ k, l }, start);                           //storing in priority array the cost alongwith manhattan distance
						q.push({ { k, l }, new_cost });																//between start and current node.Thus,pushing the pixel alomgwith its new
						img1.at<Vec3b>(k, l)[1] = 255;																//cost in the queue.Also marking the pixel visited
					}
				}
			}
			else
			{
				for (l = current.p.y - end.y; l <= current.p.y + end.y; l += end.y)								//performing the same operation from top to bottom
				{
					if (isValid(k, l, img))
					{
						new_cost = current.c + cost(k, l);
						if (img1.at<Vec3b>(k, l)[1] != 255 || (new_cost < cost_so_far[k][l]))
						{
							cost_so_far[k][l] = new_cost;
							priority[k][l] = cost_so_far[k][l] + heuristics({ k, l }, start);
							q.push({ { k, l }, new_cost });
							img1.at<Vec3b>(k, l)[1] = 255;
						}
					}
				}
			}
			/*for (k = current.p.x - 1; k <= current.p.x + 1; k++)															//same algorithm but traversing through each pixel of the 2000x2000
			{																												//image making the process too costly,by eating quite a lot of time
			for (l = current.p.y - 1; l <= current.p.y + 1; l++)
			{
			if (isValid(k, l, img) && (img.at<Vec3b>(k, l)[0] <= 240 || img.at<Vec3b>(k, l)[1] <= 240 || img.at<Vec3b>(k, l)[2] <= 240))
			{
			//new_cost = cost_so_far[current.p.x][current.p.y] + cost(k, l);
			new_cost = current.c + cost(k, l);
			if (img1.at<Vec3b>(k, l)[1] != 255 || (new_cost < cost_so_far[k][l]))
			{
			cost_so_far[k][l] = new_cost;
			priority[k][l] = cost_so_far[k][l] + heuristics({ k, l }, start);
			q.push({ { k, l }, new_cost });
			img1.at<Vec3b>(k, l)[1] = 255;
			}
			}

			}
			}*/


		}		//printf("%d-%d\n", current.p.y, current.p.x);
	}
}

point min_prior(state v[], int c)                //function to return the co-ordinate with minimum cost
{
	int k, i;
	double min = v[0].c;
	k = 0;
	for (i = 0; i < c; i++)
	{
		if (v[i].c < min)
		{
			min = v[i].c;
			k = i;
		}
	}
	return v[k].p;
}

void reconstruct_path(struct point start, struct point end,double **priority)        //function to reconstruct the path by backtracking from the end 
{																					//by finding the min cost elememnt in each pixels' 3X3 mask
	point current;                                                                  //and making the minimum cost pixel as current until current is start
	current = end;
	int k, l, i, j, sum = 0;
	img.at<Vec3b>(end.x, end.y)[2] = 255;
	while (current.x != start.x || current.y != start.y)
	{
		//state *arr = (state*)calloc(100, sizeof(state));
		state arr[100];
		int cnt = 0;
		/*for (k = current.x + 1; k >= current.x - 1; k--)                      // if traversing through each pixel(time-consuming)
			{
				for (l = current.y - 1; l <= current.y + 1; l++)
					{*/
		int c = 0;
		for (k = current.x - end.x; k <= current.x + end.x; k += end.x)
		{
			c++;
			if (c == 1 || c == 3)
			{
				l = current.y;
				if (isValid(k, l, img) && img1.at<Vec3b>(k, l)[1] == 255)								//checking if the pixel is visited by dijkstra function 
				{																						//and also if the pixel is white or not

					if ((k != current.x || l != current.y)&&
						(img.at<Vec3b>(k, l)[0] != 255 || img.at<Vec3b>(k, l)[1] != 255 || img.at<Vec3b>(k, l)[2] != 255))
					{
						state s;
						s.p = { k, l };
						s.c = priority[k][l];
						arr[cnt] = s;                                                                       //creating an array containing kernel elemnts to get the min of those elements
						cnt++;																				//and storing in state variables
						//printf("%d ? %d\n%d\n", arr[cnt].p.x, arr[cnt].p.y, cnt);
						//img1.at<Vec3b>(k, l)[2] = 255;

					}
				}
			}
			else
			{

				for (l = current.y - end.y; l <= current.y + end.y; l += end.y)
				{
					if (isValid(k, l, img) && img1.at<Vec3b>(k, l)[1] == 255)
					{

						if ((k != current.x || l != current.y) &&
							(img.at<Vec3b>(k, l)[0] != 255 || img.at<Vec3b>(k, l)[1] != 255 || img.at<Vec3b>(k, l)[2] != 255))
						{
							state s;
							s.p = { k, l };
							s.c = priority[k][l];
							arr[cnt] = s;
							cnt++;
							//printf("%d ? %d\n%d\n", arr[cnt].p.x, arr[cnt].p.y, cnt);

						}
					}
				}
			}
		}
		point current1 = min_prior(arr, cnt);                              //storing the min cost pixel.
		for (k = current1.x; k >= current.x; k--)						  //traversing from min cost vertex to previous vertex to plot the path
		{
			for (l = current1.y; l >= current.y; l--)
			{
				img.at<Vec3b>(k, l)[2] = 255;
				img2.at<Vec3b>(k, l)[2] = 255;

			}
		}current = current1;
		img.at<Vec3b>(current.x, current.y)[2] = 255;
		img2.at<Vec3b>(current.x, current.y)[2] = 255;
	}
		img.at<Vec3b>(start.x, start.y)[2] = 255;
}
void bfs(struct point start, struct point end, struct point **came_from, int **cost_so_far)
{
	point current;
	int k, l;
	queue<point> q;                                              //queue of coordinates
	int new_cost;
	q.push(start);
	came_from[start.x][start.y].x = start.x;                                  //structure to store the coordinate from which previous pixel came from
	came_from[start.x][start.y].y = start.y;
	cost_so_far[start.x][start.y] = 0;											//calcuating the cost of traversal

	while (!q.empty())
	{
		current = q.front();
		q.pop();
		img1.at<Vec3b>(current.x, current.y)[1] = 255;                         //image to check if visited or not 
		if (current.x == end.x&&current.y == end.y)
			break;																//early exit of traversal if end is found
		for (k = current.x - 1; k <= current.x + 1; k++)
		{
			for (l = current.y - 1; l <= current.y + 1; l++)                    
			{
				if (isValid(k, l, img))
				{
					new_cost = cost_so_far[current.x][current.y] + cost(k, l);                 //cost of traversal calculation
					if (img1.at<Vec3b>(k, l)[1] != 255 || (new_cost < cost_so_far[k][l])											//checking if the pixeis visted or not OR the new cost of the pixel is less than the previous coordinate
						&& (img.at<Vec3b>(k, l)[0] <= 240 || img.at<Vec3b>(k, l)[1] <= 240 || img.at<Vec3b>(k, l)[2] <= 240))
					{
						cost_so_far[k][l] = new_cost;								//storing the cost in this array
						q.push({ k, l });												//pushing the next pixel
						came_from[k][l].x = current.x;									//storing the coordinate od the pixel from which it came so that we can backtrack the path
						came_from[k][l].y = current.y;
						img1.at<Vec3b>(k, l)[1] = 255;                                 //marking the pixel visited
					}
				}
			}
		}
	}
	current = end;
	while (current.x != start.x || current.y != start.y)								//backtracking the path from end to start
	{
		current = came_from[current.x][current.y];									 //using the came_from array to get the coordinate of the parent pixel
		img.at<Vec3b>(current.x, current.y)[2] = 255;
		img2.at<Vec3b>(current.x, current.y)[0] = 255;
	}
	img.at<Vec3b>(current.x, current.y)[0] = 255;
	img2.at<Vec3b>(current.x, current.y)[0] = 255;

}

int main()
{
	int i, j;
	point start, end;	
	point **came_from;
	int **cost_so_far;
	double **priority;																//creating and initialising the 2D arrays  
	came_from = (point**)(malloc(sizeof(point*)*img.rows));
	cost_so_far = (int**)(calloc(img.rows, sizeof(int*)));
	priority = (double**)(calloc(img.rows, sizeof(double*)));
	for (i = 0; i < img.rows; i++)
	{
		came_from[i] = (point*)(malloc(sizeof(point)*img.cols));
		cost_so_far[i] = (int*)(calloc(img.cols, sizeof(int)));
		priority[i] = (double*)(calloc(img.cols, sizeof(double)));
	}																																						//calculating the centre of mass of the starting and end points
	int sum_starti = 0, sum_startj = 0, c1 = 0, c2 = 0, sum_endi = 0, sum_endj = 0;
	for (i = 0; i < img.rows; i++)																	//calculating the COM of the starting and destination
	{
		for (j = 0; j < img.cols; j++)
		{
			if (img1.at<Vec3b>(i, j)[0] != 255 && img.at<Vec3b>(i, j)[0] < 20 && img.at<Vec3b>(i, j)[1]>245 
				&& img.at<Vec3b>(i, j)[2]>245)
			{
				sum_starti += i;
				sum_startj += j;
				c1++;
				img1.at<Vec3b>(i, j)[0] = 255;
			}
			if (img1.at<Vec3b>(i, j)[0] != 255 && img.at<Vec3b>(i, j)[0] > 245 && img.at<Vec3b>(i, j)[1]>245 
				&& img.at<Vec3b>(i, j)[2]<20)
			{
				sum_endi += i;
				sum_endj += j;
				c2++;
				img1.at<Vec3b>(i, j)[0] = 255;
			}
		}
	}
	start.x = sum_starti / c1;
	start.y = sum_startj / c1;
	end.x = sum_endi / c2;
	end.y = sum_endj / c2;
	filter(start, end);                        //calling filter to remove unwanted noise
	int ch;
	printf("Press :1 for A-star\n      :2 for BFS\n");         //creating the path using desired algorithm
	scanf("%d", &ch);
	if (ch == 1)
	{
		dijkstra(start, end, cost_so_far, priority);           
		reconstruct_path(start, end, priority);
	}
	else
		bfs(start, end, came_from, cost_so_far);
	namedWindow("Optimal Path", WINDOW_AUTOSIZE);
	imshow("Optimal Path", img);
	namedWindow("Djikstra Traversal", WINDOW_NORMAL);
	imshow("Djikstra Traversal", img1);
	namedWindow("The path", WINDOW_NORMAL);
	imshow("The path", img2);
	waitKey(0);
	return 0;
}