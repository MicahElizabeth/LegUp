//add a test comment for commit practice
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Optimization;

namespace LegUpConsole
{
    class Program
    {
        static public bool GetMinOrMax()
        {
            string temp = "";
            do
            {

                Console.WriteLine("Max or Min? ");
                temp = Console.ReadLine();

            } while (temp != "Min" && temp != "Max");

            if (temp == "Max")
                return false;
            else
                return true;
        }

        static public int GetNumItems()
        {
            int i = 0;
            string temp = "";
            do
            {
                Console.Write("How many types of Items do you have? ");
                temp = Console.ReadLine();

            }while(!Int32.TryParse(temp, out i) || i <= 0);

            return i;
        }

        static public Constraint GetConstraint(List<DecisionVariable> dvs, int consCount)
        {
            string name = "";
            Console.Write("Constraint Name: ");
            name = Console.ReadLine();

            Constraint c = new Constraint(consCount);
            string temp = "";
            double tempD = 0.0;
            foreach(DecisionVariable x in dvs)
            {
                do
                {
                    Console.Write("How much {0} does {1} add/subtract from {2} total? (double value) ", name, x.Name, name);
                    temp = Console.ReadLine();
                } while (!double.TryParse(temp, out tempD));

                c.AddCoef(x, tempD);
            }

            do
            {
                Console.Write("Is the bound comparator (1)=, (2)<=, (3)>= ? ");
                temp = Console.ReadLine();
            } while (temp != "1" && temp != "2" && temp != "3");

            switch (temp)
            {
                case "1": c.Relation = Symbol.Equals; break;
                case "2": c.Relation = Symbol.Less; break;
                default: c.Relation = Symbol.Greater; break;

            }

            do
            {
                Console.Write("What is the bound value? (double value) ");
                temp = Console.ReadLine();

            } while (!double.TryParse(temp, out tempD));

            c.BValue = tempD;

            return c;
        }

        static public DecisionVariable GetDV(string objFun)
        {
            
            string temp = "";
            
            Console.WriteLine ("Enter Item Name: ");
            temp = Console.ReadLine ();
            
            DecisionVariable dv = new DecisionVariable(temp);
            double i = 0;
            do
            {
                Console.WriteLine("Enter {0} per {1}: ", objFun, temp);
                temp = Console.ReadLine();
                
            }while (!Double.TryParse(temp, out i)) ;
           
            dv.ObjectiveCoeff = i;

            return dv;
        }

        static public bool IsMoreConstraints()
        {
            string temp = "";
            do
            {
                Console.Write("Add a constraint?(y/n) ");
                temp = Console.ReadLine();
            } while (temp != "y" && temp != "n");

            if (temp == "y")
                return true;
            else
                return false;
        }


        static void Main(string[] args)
        {
            Console.WriteLine("What are you optimizing (i.e. Health, Damage, etc.)?");
            string opFun = Console.ReadLine();

            BranchAndBoundIP ip = new BranchAndBoundIP(GetMinOrMax());

            int numDecisionVariables = GetNumItems();
            List<DecisionVariable> dvs = new List<DecisionVariable>();
            
            for (int i = 0; i < numDecisionVariables; i++)
            {
                DecisionVariable temp = GetDV(opFun);
                ip.AddDV(temp);
                dvs.Add(temp);
            }

            int j = 0;
            while(IsMoreConstraints())
            {
                ip.AddConstraint(GetConstraint(dvs, j++));

            }

            j = 0;
            if (ip.Solve() == Solution.Found)
            {
                Console.WriteLine("Your Solution is:");
                foreach(Point p in ip.OptimalPoints)
                {
                    Console.WriteLine("--Solution {0}--", j++);
                    foreach (DecisionVariable x in p.PointVars)
                    {
                        Console.WriteLine("\t\t {0}: {1}", x.Name, x.Value);
                    }

                    Console.WriteLine("\t\t Optimal {0}: {1}", opFun, ip.OptimalValue);
                }
            }
            else 
            { 
                Console.WriteLine("No Solution Found"); 
            }
                

        }
    }
}
