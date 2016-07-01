using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Optimization
{
    public class BranchAndBoundIP
    {
        public List<LP> _A; //list of active LPs
        public List<Point> _B; //list of know int feasable xhats's to IP that have the value, zBar
        public Nullable<double> _zBar; // best know objective with int solution xhat nullable so that I don't have to pick and initial value
        private List<Constraint> _constraints;
        private List<DecisionVariable> _DVs;
        private bool _isMin;

        public List<Point> OptimalPoints { get { return _B; } }
        public Nullable<double> OptimalValue  //not returning the correct value
        {
            get {
                if (_isMin)
                    return _zBar;
                else
                    return (_zBar * (-1));
            } 
        }

        //constructor
        public BranchAndBoundIP(bool isMin = true)
        {
            _A = new List<LP>();
            _B = new List<Point>();
            _zBar = null; //is a nullable, no way to pick an initial value b/c it will be used to compare
            _constraints = new List<Constraint>();
            _DVs = new List<DecisionVariable>();
            _isMin = isMin;

        }

        public void AddDV(DecisionVariable var)
        {
            if(!_isMin) //convert to min by multiplying the opjective function by negative 1
            {
                var.ObjectiveCoeff *= (-1);
                
            }
            _DVs.Add(var);
    
        }

        public void AddConstraint(Constraint c)
        {
            _constraints.Add(c);
        }

        private LP CreateLP()
        {
            LP relax = new LP(false);
            //creat deep copies of decision vars
            List<DecisionVariable> copies = new List<DecisionVariable>();
            foreach(DecisionVariable d in _DVs)
            {
                copies.Add(new DecisionVariable(d));
            }

            //Add copies of Decision Variables to the relax
            foreach (DecisionVariable x in copies)
            {
                relax.AddDV(x);
            }

            List<DecisionVariable> slacksAdded = new List<DecisionVariable>();
            //create deep copies of constraints with the new decision vars
            foreach(Constraint c in _constraints)
            {
                Constraint newC = new Constraint(c, c.Relation);
                int i = 0;
                //replace the references to x with the new ones.
                foreach(DecisionVariable x in _DVs)
                {
                    double newCoeff = newC.LHS[x];
                    newC.LHS.Remove(x);
                    newC.LHS[copies[i]] = newCoeff;
                    i++;
                }
                foreach(DecisionVariable s in slacksAdded)
                {
                    if(!newC.LHS.ContainsKey(s))
                    {
                        newC.LHS[s] = 0.0;
                    }
                }

                DecisionVariable temp = relax.AddConstraint(newC);
                if(temp != null) {slacksAdded.Add(temp);}
            }
            

            return relax;
        }

        private LP CreateLPSplit(Constraint toAdd)
        {
            LP relax = new LP(false);
            //creat deep copies of decision vars
            List<DecisionVariable> copies = new List<DecisionVariable>();
            foreach (DecisionVariable d in _DVs)
            {
                copies.Add(new DecisionVariable(d));
            }

            //Add copies of Decision Variables to the relax
            foreach (DecisionVariable x in copies)
            {
                relax.AddDV(x);
            }

            //Add the new constraint that we are splitting on
            Constraint newCons = new Constraint(toAdd, toAdd.Relation);
            int j = 0;
            //replace the references to x with the new ones.
            foreach (DecisionVariable x in _DVs)
            {
                double newCoeff = newCons.LHS[x];
                newCons.LHS.Remove(x);
                newCons.LHS[copies[j]] = newCoeff;
                j++;
            }

            DecisionVariable temp = relax.AddConstraint(newCons);

            
            //create deep copies of constraints with the new decision vars
            foreach (Constraint c in _constraints)
            {
                Constraint newC = new Constraint(c, c.Relation);
                int i = 0;
                //replace the references to x with the new ones.
                foreach (DecisionVariable x in _DVs)
                {
                    double newCoeff = newC.LHS[x];
                    newC.LHS.Remove(x);
                    newC.LHS[copies[i]] = newCoeff;
                    i++;
                }
               
                    if (temp != null && !newC.LHS.ContainsKey(temp))
                    {
                        newC.LHS[temp] = 0.0;
                    }
               
            }

           
            return relax;
        }

        private List<DecisionVariable> GetNonInts(Point p)
        {
            List<DecisionVariable> nonInt = new List<DecisionVariable>();
            foreach(DecisionVariable x in p.PointVars)
            {
                //if not an int
                if((x.Value %1) != 0)
                {
                    nonInt.Add(x);
                }
            }

            return nonInt;
        }

        public Solution Solve()
        {
            Solution result = Solution.Infeasible;
            //Add LP relax to A
            _A.Add(CreateLP());
            //while A ! empty
            while (_A.Count != 0)
            {
                //remove prob from A and solve ==> if it is Solution.Found set result = Found
                LP removed = _A[_A.Count - 1];
                _A.RemoveAt(_A.Count - 1);
                if (removed.Solve() == Solution.Found)
                { 
                    result = Solution.Found;
                    //if solution is better than or equal to current
                    if (_zBar == null || removed.XStar.Value <= _zBar)
                    {
                        //if solution point is an integer solution
                        List<DecisionVariable> nonInts = new List<DecisionVariable>();
                        if ((nonInts = GetNonInts(removed.XStar)).Count == 0)
                        {
                            //if solution is strictly better
                            if (_zBar == null || removed.XStar.Value > _zBar)
                            {
                                //clear out B 
                                _B.Clear();

                            }

                            //add point to B
                            _B.Add(removed.XStar);

                            //set zBar to solution
                            _zBar = removed.XStar.Value;
                        }
                        else //split problem on one of the non integer vars
                        //and add them both to A
                        {
                            Random rand = new Random();
                            int splitIndex = rand.Next(0, nonInts.Count);

                            DecisionVariable splitOn = nonInts[splitIndex];

                            double bottomSplitVal = Math.Floor(splitOn.Value);
                            double topSplitVal = Math.Ceiling(splitOn.Value);

                            Constraint bottomConst = new Constraint(_constraints.Count + 1);
                            bottomConst.AddCoef(splitOn, 1);
                            bottomConst.BValue = bottomSplitVal;
                            bottomConst.Relation = Symbol.Less;

                            _A.Add(CreateLPSplit(bottomConst));

                            Constraint topConst = new Constraint(_constraints.Count + 1);
                            topConst.AddCoef(splitOn, 1);
                            topConst.BValue = topSplitVal;
                            topConst.Relation = Symbol.Greater;

                            _A.Add(CreateLPSplit(topConst));
                        }
                    }
                }

            }//loop back up

            //Once A is empty we have our solution.
            return result;

        }
    }
}
