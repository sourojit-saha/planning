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
    # problems = [["./map1.txt", "pi/2,pi/4,pi/2,0.785398,1.570796", "0.392699,2.356194,pi,2.8274328,pi*3/2"],
    #             ["./map2.txt", "0.392699,2.356194,3.141592", "1.570796,0.785398,1.570796"]]
    problems = [["./map2.txt", "1.8326,0.785398,4.95674,1.51844,0.977384", "1.13446,2.14675,2.25147,2.86234,1.93732"],
                ["./map2.txt", "1.51844,1.29154,4.95674,2.14675,4.01426", "0.10472,1.62316,2.49582,1.98968,4.43314"],
                ["./map2.txt", "0.366519,1.72788,2.37365,0.20944,4.10152", "1.44862,5.84685,2.26893,1.71042,1.5708"],
                ["./map2.txt", "0.314159,2.72271,0.837758,1.27409,2.23402", "1.32645,1.32645,3.83972,2.30383,0.785398"],
                ["./map2.txt", "1.54566,0.890118,2.0944,3.33358,1.27409", "0.837758,3.47321,0.0872665,1.5708,5.16617"]]
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