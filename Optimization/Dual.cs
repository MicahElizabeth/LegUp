using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using MathNet.Numerics.LinearAlgebra.Double;

namespace Optimization
{
    class Dual
    {
        // C => bprime
        // b => Cprime
        // change the constraint symbols
        // pA  => new constraint
        // and sign constraints

        private Matrix _cTrans;
        private Matrix _b;
        private Matrix _A;
        private Matrix _signCons;
        private Matrix _ASymbols; // 1 >= 2<= 3== , [4 free] onl for signs
        private Matrix _signSymbols;
        private Matrix _signB;
        private List<DecisionVariable> _DVs;
        private int _numDV;
        private int _numCons;

        //constructor
        public Dual(List<DecisionVariable> dvs, List<List<Double>> a, List<Double> b, List<Double> consSyms,
            List<Double> snCons, List<Double> snSyms)
        {

            _cTrans = new DenseMatrix(1);
            foreach (DecisionVariable d in dvs)
            {
                Double[] ar = {d.ObjectiveCoeff};
                Matrix ap = new DenseMatrix(1,1, ar);
                _cTrans = (Matrix)_cTrans.Append(ap);
                _numDV++;
            }
            
            //Constraints
            _A = new DenseMatrix(1);
            foreach (List<Double> l in a)
            {
                Matrix rowa = new DenseMatrix(1);
                foreach (Double d in l)
                {
                    Double[] ar = { d };
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    rowa = (Matrix)rowa.Append(ap);
                }
                _A = (Matrix)_A.Append(rowa);

                _numCons++;
            }

            _b = new DenseMatrix(1); 
            int i = 0;
            foreach(Double d in b)
            {
                Double[] ar = { d };
                Matrix ap = new DenseMatrix(1, 1, ar);
                _b = (Matrix)_b.InsertRow(i++, ap.Row(0));
            }

            _ASymbols = new DenseMatrix(1,1);
            foreach(Double d in consSyms)
            {
                Double[] ar = {d};
                Matrix ap = new DenseMatrix(1, 1, ar);
                _ASymbols = (Matrix) _ASymbols.Add(ap);
            }

            /// Add sign constraints
            _signCons = new DenseMatrix(1);
            foreach (Double d in snCons)
            {
                Double[] ar = { d };
                Matrix ap = new DenseMatrix(1, 1, ar);
                _signCons = (Matrix)_signCons.Append(ap);
            }
            

            _signB = new DenseMatrix(1); 
            i = 0;
            foreach (var d in dvs)
            {
                Double[] ar = { 0 };
                Matrix ap = new DenseMatrix(1, 1, ar);
                _signB = (Matrix)_signB.InsertRow(i++, ap.Row(0));
            }

            _signSymbols = new DenseMatrix(1, 1);
            foreach (Double d in snSyms)
            {
                Double[] ar = { d };
                Matrix ap = new DenseMatrix(1, 1, ar);
                _signSymbols = (Matrix)_signSymbols.Add(ap);
            }
            
            //Refs to decisionVariables
            _DVs = dvs;
            
            
            
        }

        //copy constructor
        public Dual(Matrix cTrans, Matrix b, Matrix A, Matrix signCons,
            Matrix ASymbols, Matrix signSymbols, Matrix signB, List<DecisionVariable> DVs,
        int numDV, int numCons)
        {
            _cTrans = cTrans;
            _b = b;
            _A = A;
            _signCons = signCons;
            _ASymbols = ASymbols;
            _signSymbols = signSymbols;
            _signB = signB;
            _DVs = DVs;
            _numDV = numDV;
            _numCons = numCons;
        }


        // C => bprime
        // b => Cprime
        // change the constraint symbols
        // pA  => new constraint
        // and sign constraints

        public Dual Transform()
        {
            
            int numdv = _numCons;
            
            //pTrans in signCons
            Matrix pTrans = new DenseMatrix (1, numdv);
            List<DecisionVariable> ps = new List<DecisionVariable>();

            for(int i = numdv; i > 0; i--)
            {
                Double[] ar = { 1 };
                Matrix ap = new DenseMatrix(1, 1, ar);
                pTrans = (Matrix)pTrans.Append(ap);
            }

            Matrix cT = new DenseMatrix(1, _numCons);
            cT = (DenseMatrix)_b.Transpose();

            for (int i = 0; i < numdv; i++)
            {
                DecisionVariable dv = new DecisionVariable ("P" + i.ToString());
                dv.ObjectiveCoeff = cT.Row(0).At(i);

                ps.Add(dv);
            }

            Matrix b = new DenseMatrix(_numDV,1);
            b = (DenseMatrix)_cTrans.Transpose();

            Matrix a = new DenseMatrix(_numCons, numdv);

            a = (Matrix)pTrans.Multiply((Matrix)_A);

            //do the symbols for each dv in Prime to A symbols in Dual
            Matrix aSyms = new DenseMatrix(1, 1);
            for (int i = 0; i < _numDV; i++)
            {
                double sym = _signSymbols.Row(0).At(i);
                if(sym == 1) //x>= 0  ===> a <= b
                {
                    Double[] ar = { 2 };
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    aSyms = (Matrix)aSyms.Append(ap);
                   
                }
                else if (sym == 2)
                {
                    Double[] ar = { 1};
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    aSyms = (Matrix)aSyms.Append(ap);
                }
                else if(sym == 3)
                {
                    Double[] ar = { 4};
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    aSyms = (Matrix)aSyms.Append(ap);
                }
                else
                {
                    Double[] ar = { 3 };
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    aSyms = (Matrix)aSyms.Append(ap);
                }

                 
            }

            //Create the Duals Sign symbols
            Matrix snSyms = new DenseMatrix(1, 1);
            for (int i = 0; i < _numCons ; i++)
            {
                double sym = _signSymbols.Row(0).At(i);
                if (sym == 1) //a>=b ====> x >= 0
                {
                    Double[] ar = {1 };
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    snSyms = (Matrix)snSyms.Append(ap);

                }
                else if (sym == 2)
                {
                    Double[] ar = { 2};
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    snSyms = (Matrix)snSyms.Append(ap);
                }
                else if (sym == 3)
                {
                    Double[] ar = { 4};
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    snSyms = (Matrix)snSyms.Append(ap);
                }
                else
                {
                    Double[] ar = { 3};
                    Matrix ap = new DenseMatrix(1, 1, ar);
                    snSyms = (Matrix)snSyms.Append(ap);
                }
                
            }

            //set each sign constraint's b value
            Matrix snB = new DenseMatrix(1, 1);
            for (int i = 0; i < _numDV; i ++)
            {
                Double[] ar = { 0 };
                Matrix ap = new DenseMatrix(1, 1, ar);
                snB = (Matrix)snB.Append(ap);
            }


            Dual newDual = new Dual(cT, b, a, pTrans, aSyms, snSyms, snB, ps, _numCons, _numDV);

            return newDual;
        }

        public List<DecisionVariable> solve(Dual d)
        {
            //put all the pices into one big matrix and solve the sytem of linear inequalities

            Matrix A = _A;
            for (int i = 0; i < d._numCons; i ++)
            {
                // create a giant a matrix with A O        sign stuff b
                //                              O d._A     sign stuff d.b
                //                              
            }

            //then solve it

            return new List<DecisionVariable>();
        }
    }


    
}
