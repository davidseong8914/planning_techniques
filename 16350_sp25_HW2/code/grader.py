import numpy as np
import pdb
import argparse
import subprocess # For executing c++ executable
import pandas as pd
from timeit import default_timer as timer

###############################################################
################### Util Functions Below ######################

def convertPIs(aString):
    """ Input: A comma seperated string like "pi/2,pi/4,pi/2,pi/4,pi/2,"
            or 1.2,4,5.3 etc
    Output: string replacing the pis 1.57079,...,...
    """
    if aString[-1] == ",":  # Remove training comma if there is one
        aString = aString[:-1]
    aString = aString.replace("pi", "3.141592") # Replace pi with 3.14... if needed
    vecOfStrings = aString.split(",")
    ans = []
    for anExpression in vecOfStrings:
        ans.append(str(eval(anExpression))) # Evaluate expressions if needed
    return ans

###############################################################
################### Main Functions Below ######################


def graderMain(executablePath, gradingCSV):
    problems = [["./map1.txt", "pi/2,pi/4,pi/2,0.785398,1.570796",
                                "0.392699,2.356194,pi,2.8274328,pi*3/2"],
            ["./map2.txt", "0.392699,2.356194,3.141592",
                                "1.570796,0.785398,1.570796"]]
    # problems = [
        # # 3 DoF
        # ["./map2.txt", "0.392699,2.356194,3.141592",
        #                     "1.570796,0.785398,1.570796"],
        # # 4 DoF
        # ["./map2.txt", "0.87,5.62,3.27,0.76",
        #                     "1.14,2.19,3.81,0.16"],
        # 5 DoF
        # ["./map2.txt", "1.00,2.51,6.10,5.24,2.93",
        #                         "1.32,3.12,6.15,1.65,3.27"],
        # # 6 DoF
        # ["./map2.txt", "1.02,5.70,3.44,0.84,2.01,4.61",
        #                     "1.18,1.37,4.83,2.94,1.52,4.64"],
        # 7 DoF
        # ["./map2.txt", "1.44,2.10,0.06,2.92,6.03,1.76,6.10",
        #                     "0.65,0.36,1.83,2.45,3.91,5.56,3.09"]
                            # ]
    scores = []
    for aPlanner in [0, 1, 2, 3]:
        for i, data in enumerate(problems):
            inputMap, startPos, goalPos = [*data]
            numDOFs = len(startPos.split(","))
            outputSolutionFile = "tmp.txt"
            startPosString = ",".join(convertPIs(startPos))
            goalPosString = ",".join(convertPIs(goalPos))
            commandPlan = "{} {} {} {} {} {} {}".format(
                executablePath,
                inputMap, numDOFs, startPosString, goalPosString,
                aPlanner, outputSolutionFile)
            commandVerify = "./verifier.out {} {} {} {} {}".format(
                inputMap, numDOFs, startPosString, goalPosString,
                outputSolutionFile)
            try:
                start = timer()
                subprocess.run(commandPlan.split(" "), check=True) # True if want to see failure errors
                timespent = timer() - start
                returncode = subprocess.run(commandVerify.split(" "), check=False).returncode
                if returncode != 0:
                    print("Returned an invalid solution")
                
                ### Calculate the cost from their solution
                with open(outputSolutionFile) as f:
                    line = f.readline().rstrip()  # filepath of the map
                    solution = []
                    for line in f:
                        solution.append(line.split(",")[:-1]) # :-1 to drop trailing comma
                    solution = np.asarray(solution).astype(float)
                    numSteps = solution.shape[0]

                    ## Cost is sum of all joint angle movements
                    difsPos = np.abs(solution[1:,]-solution[:-1,])
                    cost = np.minimum(difsPos, np.abs(2*np.pi - difsPos)).sum()

                    success = returncode == 0
                    scores.append([aPlanner, inputMap, i, numSteps, cost, timespent, success])
            
                ### Visualize their results
                commandViz = "python visualizer.py tmp.txt --gifFilepath=grades_{}.gif".format(i)
                commandViz += " --incPrev=1"
                subprocess.run(commandViz.split(" "), check=True) # True if want to see failure errors
            except Exception as exc:
                print("Failed: {} !!".format(exc))
                scores.append([aPlanner, inputMap, i, -1, -1, timespent, False])

    ### Save all the scores into a csv to compute overall grades
    df = pd.DataFrame(scores, columns=["planner", "mapName", "problemIndex", "numSteps", "cost", "timespent", "success"])
    df.to_csv(gradingCSV, index=False)
            

if __name__ == "__main__":
    graderMain("./planner.out", "test.csv")