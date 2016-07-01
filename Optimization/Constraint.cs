using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Optimization
{
    
    public enum Symbol { Equals, Greater, Less }

    public class Constraint
    {
        private Dictionary<DecisionVariable, double> _lhs;
        private double _rhs; //b value
        private int _constraintNumber;
        private Symbol _relation;
        public Symbol Relation { get { return _relation; } set { _relation = value; } }

        public double GetCoef(DecisionVariable var)
        {
            return _lhs[var];
        }

        public void AddCoef(DecisionVariable var, double coef)
        {
            _lhs[var] = coef;
        }

        public double BValue { get { return _rhs; } set { _rhs = value; } }
        public Dictionary<DecisionVariable, double> LHS { get { return _lhs; } }

        //constructor
        public Constraint (int constraintNumber)
        {
            _lhs = new Dictionary<DecisionVariable, double>();
            _rhs = 0.0;
            _relation = Symbol.Equals;
            _constraintNumber = constraintNumber;
        }
        //copy constructor wiht symbol
        public Constraint (Constraint c, Symbol sym)
        {
            _relation = sym;
            _lhs = new Dictionary<DecisionVariable,double>(c._lhs);
            _rhs = c._rhs;
            _constraintNumber = c._constraintNumber;
        }

        public Constraint ()

        //adds slack variables and makes b positive
        public DecisionVariable Standardize()
        {
            if (_relation == Symbol.Greater)
            {
                //add a slack variable with +1 coeff
                DecisionVariable slack = new DecisionVariable("s" + _constraintNumber.ToString());
                _lhs[slack]= -1.0;
                slack.ObjectiveCoeff = 0.0;
                _relation = Symbol.Equals;
                if(_rhs < 0)
                {
                    foreach(DecisionVariable x in _lhs.Keys)
                    {
                        _lhs[x] = _lhs[x] * (-1);
                    }

                    _rhs = _rhs * (-1);

                }
                return slack;

            }
            else if(_relation == Symbol.Less)
            {
                DecisionVariable slack = new DecisionVariable("s" + _constraintNumber.ToString());
                _lhs[slack]= 1.0;
                slack.ObjectiveCoeff = 0.0;
                _relation = Symbol.Equals;
                if (_rhs < 0)
                {
                    foreach (DecisionVariable x in _lhs.Keys)
                    {
                        _lhs[x] = _lhs[x] * (-1);
                    }

                    _rhs = _rhs * (-1);

                }
                return slack;
            }
            else
            {
                if (_rhs < 0)
                {
                    foreach (DecisionVariable x in _lhs.Keys)
                    {
                        _lhs[x] = _lhs[x] * (-1);
                    }

                    _rhs = _rhs * (-1);

                }
            }

            return null;

            
        }

    }
}
