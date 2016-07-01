using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Optimization
{
    public enum Solution { Found, Unbounded, Infeasible }
    //must be a min
    public class LP
    {
        private Matrix _cTrans;
        private Matrix _cBarTrans;
        private Matrix _A; // will be m by n
        private Matrix _b; // will be m by 1
        private Double _negZ;
        private List<DecisionVariable> _Basis;
        private List<DecisionVariable> _NullSpace; //might not need...
        private int _spaceDim;
        private int _numConstraints;
        private List<Constraint> _constraints;
        private HashSet<Point> _PointsSeen;
        private Point _xStar;
        private List<DecisionVariable> _DVs;
        private List<DecisionVariable> _slackVs;
        private bool _isAux;

        public Point XStar { get { return _xStar; } }

        //constructor
        public LP(bool isAux)
        {

            //_cTrans = new DenseMatrix(1,1);
            //_cBarTrans = new DenseMatrix(1, 1);
            //_A = new DenseMatrix(0, 0);
            //_b = new DenseMatrix(0, 0); // will be m by 1
            _negZ = 0.0;
            _Basis = new List<DecisionVariable>();
            _NullSpace = new List<DecisionVariable>();
            _spaceDim = 0;
            _numConstraints = 0;
            _constraints = new List<Constraint> ();
            _PointsSeen = new HashSet<Point>();
            _xStar = new Point();
            _DVs = new List<DecisionVariable>();
            _slackVs = new List<DecisionVariable>();
            _isAux = isAux;
        }
        public LP(int n, int m, List<DecisionVariable> dvs, List<Constraint> cons, bool isAux)
        {

            _cTrans = new DenseMatrix(1,n);
            _cBarTrans = new DenseMatrix(1,n);
            _A = new DenseMatrix(m,n);
            _b = new DenseMatrix(m,1); // will be m by 1
            _negZ = 0.0;
            _Basis = new List<DecisionVariable>();
            _NullSpace = new List<DecisionVariable>();
            _spaceDim = n;
            _numConstraints = m;
            _constraints = cons;
            _PointsSeen = new HashSet<Point>();
            _xStar = new Point();
            _DVs = dvs;
            _slackVs = new List<DecisionVariable>();
            _isAux = isAux;
        }


        public void AddDV(DecisionVariable var)
        {
            _spaceDim++;
            _DVs.Add(var);
            Double[] ar = {var.ObjectiveCoeff };
            Matrix ap = new DenseMatrix(1, 1, ar);
            if (_cTrans == null)
            {
                _cTrans = ap;
            }
            else
            {
                _cTrans = (Matrix)_cTrans.Append(ap);
            }
           
        }

        //must not be called until all Dvars are added
        public DecisionVariable AddConstraint(Constraint c)
        {
           
            DecisionVariable x = c.Standardize();
            if (x != null)
            {
                _slackVs.Add(x);
                //add to all constraints
                foreach(Constraint a in _constraints)
                {
                    a.AddCoef(x, 0.0);
                    
                }
            }
            _spaceDim++;
            _numConstraints++;
            Double[] ar = {0.0};
            Matrix ap = new DenseMatrix(1,1,ar);
            
            _cTrans = (Matrix) _cTrans.Append(ap);
            
            

            _constraints.Add(c);

            return x;
        }
        //add all the constrain coeffs to A and values to b
        private void InitializeAand_b()
        {
            _A = new DenseMatrix(_numConstraints, _spaceDim);
            _b = new DenseMatrix(_numConstraints, 1);
            int i = 0;
            foreach(Constraint c in _constraints)
            {
                int j = 0;
                Double[] doubles = new Double[_spaceDim];
                foreach (DecisionVariable x in c.LHS.Keys)
                {
                    doubles[j] = c.LHS[x];
                    j++;
                }
                
                _A.SetRow(i,doubles);

                Double[] doubles2 = new Double[1]{c.BValue};
                _b.SetRow(i, doubles2);
                i++;
            }
        }

        private void InitializeCBarTrans()
        {
            
            //first calculate CTrans_B * A_Binverse
            
            Matrix a_Binverse = new DenseMatrix(_A.RowCount, _Basis.Count);
            Double[] doubles1 = new Double[_Basis.Count];
            

            int i = 0;

            foreach (DecisionVariable x in _Basis)
            {
                 doubles1[i] = x.ObjectiveCoeff;
                
                Double[] doubles2 = new Double[_A.RowCount];
                int k = 0;
                foreach(Constraint c in _constraints)
                {
                    doubles2[k] = c.GetCoef(x);
                    k++;
                }

                a_Binverse.SetColumn(i,doubles2);

                i++;
            }

            Matrix cTrans_B = new DenseMatrix(1,_Basis.Count, doubles1);
            
                //invert a_binverse
            a_Binverse = (Matrix) a_Binverse.Inverse();


            Matrix result =(Matrix) (cTrans_B*a_Binverse);

            _cBarTrans = (Matrix)(_cTrans - (result * _A));
            
        }

        private void CalculateNegZAndXStar()
        {
            foreach (DecisionVariable x in _Basis)
            {
                _xStar.addVar(x);
            }
            foreach(DecisionVariable x in _slackVs)
            {
                _xStar.addVar(x);
            }

            Vector xhat = new DenseVector(_spaceDim);
            int i = 0;
            foreach (DecisionVariable x in _xStar.PointVars)
            {
                xhat[i] = x.Value;
                i++;
            }
            _xStar.Value = _cTrans.Row(0) * xhat;
            _negZ = _xStar.Value * (-1);
        }
        private bool IsFeasible(Point p)
        {
            double res = 0.0;
            foreach(Constraint c in _constraints)
            {
                foreach (DecisionVariable x in p.PointVars)
                {
                    res += c.LHS[x] * x.Value;
                }

                if(res != c.BValue)
                {
                    return false;
                }
            }

            return true;
        }
        
        private bool SeenPoint()
        {
            bool res = false;
            foreach(Point p in _PointsSeen)
            {
                int i = 0;
                foreach(DecisionVariable x in p.PointVars)
                {
                    if(x.Value != _xStar.PointVars[i].Value)
                    {
                        res = false;
                        break;
                    }
                    else
                    {
                        res = true;
                    }
                    
                    i++;
                }

                if(res == true)
                {
                    return true;
                }

            }

            return false;
        }
    
        //return true if we can find one false otherwise(infeasible)
        private bool FindInitialBasisAndNullSpace()
        {
            //always have an auxillary so that we can test for infeasibility
            // and whatnot.
            if(_isAux)
            {
                //add all our decision variables 
                //(note we do this so that they are in the correct order in the xstar point)
                foreach (DecisionVariable x in _DVs)
                {
                    x.Value = 0.0;
                    _NullSpace.Add(x);
                    _xStar.addVar(x);
                }
                //THEN add all the slack variables (again so that all the xstars are in the right order)
                for(int m = 0; m < _numConstraints; m ++)
                {
                    _slackVs[m].Value = _b[m,0];
                    _Basis.Add(_slackVs[m]);
                    _xStar.addVar(_slackVs[m]);

                }

                

                return true;
            }
            else
            {
                //create auxillary problem and solve
                LP aux = new LP(true);
                foreach(DecisionVariable x in _DVs)
                {
                    aux.AddDV(x);
                }
                foreach(DecisionVariable s in _slackVs)
                {
                    aux.AddDV(s);
                }
                Double[] newcTrans = new Double[aux._spaceDim + _numConstraints];
                
                int i = 0;
                for( i = 0; i < (aux._spaceDim) ; i++)
                {
                    newcTrans[i] = 0.0;
                }

                List<DecisionVariable> slacksAdded = new List<DecisionVariable>();
                foreach(Constraint c in _constraints)
                {
                    //set to less so that we get slacks
                    //for each constraint make a new constraint so that our original constraints aren't altered
                    Constraint auxCons = new Constraint(c, Symbol.Less);
                    foreach (DecisionVariable s in slacksAdded)
                    {
                        if (!auxCons.LHS.ContainsKey(s))
                        {
                            auxCons.LHS[s] = 0.0;
                        }
                    }

                    DecisionVariable temp = aux.AddConstraint(auxCons);
                    if (temp != null) { slacksAdded.Add(temp); }
                    
                    newcTrans[i] = 1;
                    i++;
                }
                //note, aux.spaceDim gets update in the foreach
                //objective function is the sum of all the new slacks
                aux._cTrans = new DenseMatrix(1, aux._spaceDim , newcTrans);

                if (aux.SolveAux())
                {
                    _Basis = aux._Basis;
                    _NullSpace = aux._NullSpace;
                    _A = aux._A;
                    _b = aux._b;
                    _xStar = aux._xStar;


                    return true;
                }
                else
                {
                    return false;
                }
              
            }   
            
        }

        private List<int> AuxInBasis()
        {
            List<int> indecies = new List<int>();
            int i = 0;
            foreach (DecisionVariable x in _Basis)
            {
                if(_slackVs.Contains(x))
                {
                    indecies.Add(i);
                }
                i++;
            }

            return indecies;
        }

        //returns false if infeasible
        private bool SolveAux()
        {
            Solution sol = Solve();
            
            if (sol != Solution.Found || _xStar.Value != 0) { return false; }

            //Drive out the Aux vars
            //while(there are aux in the basis)
            List<int> indecies = new List<int>();
            Random rnd = new Random();
            int i = 0, j = 0;
            while ((indecies = AuxInBasis()).Count != 0)
            {
                //1 find any aux vars in the basis
                j = indecies[rnd.Next(0, indecies.Count)];
                //i = _xStar.PointVars.IndexOf(_Basis[j]);
                //2 if the non aux entries in the row are 0,
                //we need to remove that row (from A and b) becuase it is redundant
                int k = 0;
                for (k = 0; k < _DVs.Count; k++)
                {
                    if(_A[j,k] != 0)
                    {
                        break;
                    }
                }
                if(k ==_DVs.Count)//there is a redundant row
                {
                    _Basis.RemoveAt(j);
                    _A = (Matrix)_A.RemoveRow(j);
                    _b = (Matrix)_b.RemoveRow(j);
                }
                else if (_A[j, k] != 0)//3 else if the (i=j= aux in basis) is not zero
                    //do a pivot with i=j as the pivot
                {
                    //k = GetPivotRowIndex(j);
                    //Pivot(k, j);
                    Pivot(j, k);
                }
            }
           //remove auxilary vars
            
            foreach(DecisionVariable s in _slackVs)
            {
                //remove from Null, A cols, XStar (should already be removed from A row and basis)
                _NullSpace.Remove(s);
                _A = (Matrix)_A.RemoveColumn((_A.ColumnCount - 1));
                _xStar.PointVars.Remove(s);
                
            }

            return true;
        }

        //returns a listof the potential pivot indicies, if none, empty list
        private List<int> GetPivotOptions()
        {
            List<int> pivs = new List<int>();
            for (int i = 0; i < _cBarTrans.ColumnCount; i++)
            {
                if(_cBarTrans[0,i] < 0)
                {
                    pivs.Add(i);
                }
            }

            return pivs;
        }

        private int GetPivotRowIndex(int j)
        {
            
            int i = 0;
            int minInd = -1;
            double min = 0;
            double temp = 0;

            do
            {
               
                if(_A[i,j] > 0)
                {
                    if (minInd == -1) //set the initial min
                    {
                        min = (_b[i, 0] * (-1)) / (_A[i, j] * (-1));
                        minInd = i;
                    }
                    else
                    {
                        temp = (_b[i, 0] * (-1)) / (_A[i, j] * (-1));
                        if (temp < min)
                        {
                            min = temp;
                            minInd = i;
                        }
                    }
                    
                    
                }

                i++;
            }while(i < _b.RowCount);

            return minInd;
        }

        //pivots on the ij th element of A
        private void Pivot(int i, int j) //i is row, j is col of pivot
        {
            //set i,j to 1
            double pivVal = _A[i, j];
            _A.SetRow(i, _A.Row(i) / pivVal);
            _b.SetRow(i, _b.Row(i) / pivVal);

            //zero out CBarTrans j and update b
            double cBarVal = _cBarTrans[0, j];
            _cBarTrans.SetRow(0,( _cBarTrans.Row(0) + (_A.Row(i) * (-1)*(cBarVal))));
            _negZ = _negZ + (_b[i, 0] * (-1) * (_negZ));
            
            //zero out all entries above and below row i in col j
            int k = 0;
            foreach (Double d in _A.Column(j))
            {
                if (k != i)
                {
                    double val = _A[k, j];

                    _A.SetRow(0, (_A.Row(k) + (_A.Row(i) * (-1) * (val))));
                    _b[k,0] = _b[k,0] + (_b[i, 0] * (-1) * (_b[k,0]));
                }
                k++;
            }

            //replace Element in basis with pivot element
            int index = _NullSpace.IndexOf(_xStar.PointVars[j]);
            DecisionVariable temp = _Basis[i];
            temp.Value = 0.0;
            _Basis[i] = _NullSpace[index];
            _NullSpace[index] = _xStar.PointVars[j];

            
            k=0;
            foreach(Double d in _b.Column(0))
            {
                _Basis[k].Value = d;
                    k++;
            }

            _xStar.Value = _negZ * (-1);

        }

        public Solution Solve()
        {
            //calculate all the pieces
            InitializeAand_b();
            
            //choose the first m variables to be in the basis
            if(FindInitialBasisAndNullSpace())
            {
                InitializeCBarTrans();
                List<int> pivs = new List<int>();
                Random rnd = new Random();
                _PointsSeen.Add(new Point(_xStar));

                while ((pivs = GetPivotOptions()).Count != 0)
                {
                    int i = 0, j = 0;
                    //find pivot
                    j = pivs[rnd.Next(0, pivs.Count)];
                    i = GetPivotRowIndex(j);
                   if(i == -1) //all entries are negative, so unboundend in that direction
                   {
                       return Solution.Unbounded; 
                   }

                    //pivot
                    Pivot(i, j);
                    if(SeenPoint())
                    {
                        break; //found optimal && degenerate;
                    }
                    else
                    {
                        _PointsSeen.Add(new Point(_xStar));
                    }
                }

                //we have found an optimal point which is xStar YAY!
            }
            else { return Solution.Infeasible; }

            return Solution.Found;
        }

    }
}
