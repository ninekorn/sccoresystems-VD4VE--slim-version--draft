using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
//using System.Windows.Forms;
using System.Xml;
using System.Drawing.Imaging;
using System.Drawing.Drawing2D;
using System.Runtime.InteropServices;
using System.Reflection;
using Microsoft.Win32.SafeHandles;
using System.Diagnostics;

namespace WindowsFormsApp2
{
    public partial class Form1// : Form
    {

        public int numberOfXMLToCreate;
        private static readonly Random newRandom = new Random();
        Random rnd = new Random();
        private static readonly Random getrandom = new Random();
        private static readonly object syncLock = new object();

        public Form1()
        {
            //InitializeComponent();
        }

        private void button1_Click(object sender, EventArgs e)
        {
            System.Globalization.CultureInfo customCulture = (System.Globalization.CultureInfo)System.Threading.Thread.CurrentThread.CurrentCulture.Clone();
            customCulture.NumberFormat.NumberDecimalSeparator = ".";
            System.Threading.Thread.CurrentThread.CurrentCulture = customCulture;

            for (int i = 0; i < numberOfXMLToCreate; i++)
            {               
                string path = "c:\\Users\\ninekorn\\Desktop\\testXML\\" + i + ".xml";
                XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
                writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");
               
                writer.Formatting = Formatting.Indented;
                writer.Indentation = 2;

                writer.WriteStartElement("rooting");
                writer.WriteStartElement("rooter");
                writer.WriteString(i + "");
                writer.WriteEndElement();
                writer.WriteEndElement();
             
                writer.Close();
            }
            //MessageBox.Show("XML File(s) created!");
        }
        private void createNode(string pID, string pName, string pPrice, XmlTextWriter writer)
        {
            //writer.WriteStartElement("header");
            //writer.WriteStartElement("gfx");
            //writer.WriteStartElement("data");
            writer.WriteStartElement(pName);
            writer.WriteString(pID);
            writer.WriteEndElement();

            //writer.WriteString(pID);
            //writer.WriteEndElement();

            //writer.WriteStartElement("Product_name");
            //writer.WriteString(pName);
           /// writer.WriteEndElement();
            //writer.WriteStartElement("Product_price");
            //writer.WriteString(pPrice);
            //writer.WriteEndElement();
            //writer.WriteEndElement();
        }
        public double NextFloat(float min, float middle, float max)
        {
            return rnd.NextDouble() * (max - min) + min;
        }          

        public static int GetRandomNumber(int min, int max)
        {
            lock (syncLock)
            { // synchronize
                return getrandom.Next(min, max);
            }
        }

        private void numericUpDown1_ValueChanged(object sender, EventArgs e)
        {

            //numberOfXMLToCreate = (int)numericUpDown1.Value;

            //MessageBox.Show(numericUpDown1.Value + " ");



            /*if (numericUpDown1.DecimalPlaces > 0)
            {
                numericUpDown1.DecimalPlaces = 0;
                numericUpDown1.Value = Decimal.Round(numericUpDown1.Value, 0);
            }
            else
            {
                numericUpDown1.DecimalPlaces = 2;
                numericUpDown1.Increment = 0.25M;
            }*/
            //numberOfXMLToCreate
        }
    }
}















/*static double NextFloat(float min,float middle,float max)
      {
          //double mantissa = (random.NextDouble() * 2.0) - 1.0;
          //double exponent = Math.Pow(2.0, random.Next(-126, 128));
          //return (float)(mantissa * exponent);

          Random rnd = new Random();
          double randomFloat = 0;

          for (var ctr = min; ctr < max; ctr++)
          {
              //randomFloat = ctr + rnd.NextDouble();
              randomFloat = ctr+(rnd.NextDouble() * (max - min) + min);

          }

          return randomFloat;
      }*/



//writer.WriteStartDocument(true);
//writer.WriteEndDocument();




//createNodeHeader(i + "", " This is an Energy Weapon from the" + " "  +".", "Energy Weapon", "1", writer);
/*private void createNodeHeader(string pID, string pTitle, string pDescription, string enabled, XmlTextWriter writer)
{
    writer.WriteStartElement("header");
    writer.WriteStartElement("id");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("title");
    writer.WriteString(pTitle);
    writer.WriteEndElement();
    writer.WriteStartElement("description");
    writer.WriteString(pDescription);
    writer.WriteEndElement();
    writer.WriteStartElement("enabled");
    writer.WriteString(enabled);
    writer.WriteEndElement();
    writer.WriteEndElement();
}*/















/*private void button2_Click(object sender, EventArgs e)
{
    Image firstImage0 = Image.FromFile(@"C:\Users\ninekorn\Desktop\LAYERS\PNG\DARKBLUE.png");
    Image firstImage1 = Image.FromFile(@"C:\Users\ninekorn\Desktop\LAYERS\PNG\BULLETION.png");
    Image firstImage2 = Image.FromFile(@"C:\Users\ninekorn\Desktop\LAYERS\PNG\WEAPONCIVILIAN.png");

    var target = new Bitmap(firstImage0.Width, firstImage0.Height, PixelFormat.Format32bppArgb);
    var graphics = Graphics.FromImage(target);

    int newWidth = 128;
    int newHeight = 128;

    Bitmap bmp = new Bitmap(firstImage1);

    for (int x = 0; x < bmp.Width; x++)
    {
        for (int y = 0; y < bmp.Height; y++)
        {
            Color gotColor = bmp.GetPixel(x, y);

            if (gotColor != Color.FromArgb(0, 0, 0, 0))
            {

                int a = 255;
                int r = gotColor.R;
                int g = gotColor.G;
                int b = gotColor.B;

                Color newColor = Color.FromArgb(a, r, g, b);
                bmp.SetPixel(x, y, newColor);
            }
        }
    }

    var graphics0 = Graphics.FromImage(firstImage0);
    var graphics1 = Graphics.FromImage(bmp);
    var graphics2 = Graphics.FromImage(firstImage2);

    graphics.DrawImage(firstImage0, new Rectangle(0, 0, newWidth, newHeight));
    graphics.DrawImage(bmp, new Rectangle(0, 0, newWidth, newHeight));
    graphics.DrawImage(firstImage2, new Rectangle(0, 0, newWidth, newHeight));

    target.Save(@"C:\Users\ninekorn\Desktop\LAYERS\PNG\newImage.png", System.Drawing.Imaging.ImageFormat.Png);
}*/







/*private string chooseBulletShotMode()
{
    weaponSize = new string[]
    {
        "bur",
        "one",
        "aut",
    };

    int arrayLength = weaponSize.Length;
    int index = GetRandomNumber(0, arrayLength);
    return weaponSize[index];
}*/







/*private bool predicate_FileMatch(string fileName)
       {
           if (fileName.EndsWith(".png"))
               return true;
           return false;
       }*/







/*private void button1_Click(object sender, EventArgs e)
{

    string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML18.xml";

    XmlWriterSettings settings = new XmlWriterSettings();
    settings.Encoding = Encoding.UTF8;
    settings.Indent = true;
    XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
    //writer.WriteStartDocument(true);
    writer.Formatting = Formatting.Indented;
    writer.Indentation = 2;
    writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");

    writer.WriteStartElement("root");
    createNode("1", "header", "", writer);
    writer.WriteEndElement();
    createNode("1", "gfx", "", writer);
    writer.WriteEndElement();
    createNode("3", "data", "", writer);
    writer.WriteEndElement();
    writer.Close();
    MessageBox.Show("XML File created ! ");

    /*createNode("1", "header", "", writer);
    writer.WriteEndElement();
    createNode("2", "gfx", "", writer);
    writer.WriteEndElement();
    createNode("3", "data", "", writer);
    writer.WriteEndElement();
    //writer.WriteEndElement();
    //writer.WriteEndDocument();
    writer.Close();
    MessageBox.Show("XML File created ! ");
}
private void createNode(string pID, string pName, string pPrice, XmlTextWriter writer)
{
    writer.WriteStartElement("header");
    writer.WriteStartElement("gfx");
    writer.WriteStartElement("data");

    writer.WriteString(pID);
    writer.WriteEndElement();

    //writer.WriteString(pID);
    //writer.WriteEndElement();

    writer.WriteStartElement("Product_name");
    writer.WriteString(pName);
    writer.WriteEndElement();
    writer.WriteStartElement("Product_price");
    writer.WriteString(pPrice);
    writer.WriteEndElement();
    writer.WriteEndElement();
}*/















//XmlDeclaration xmldecl;
//xmldecl = newDocument.CreateXmlDeclaration("1.0", null, null);
//xmldecl.Encoding = "UTF-8";

/*XmlDocument doc = new XmlDocument();
string xmlString = "<book><title>Oberon's Legacy</title></book>";
doc.Load(new StringReader(xmlString));

XmlDeclaration xmldecl;
xmldecl = doc.CreateXmlDeclaration("1.0", null, null);
xmldecl.Encoding = "UTF-8";*/



// Add the new node to the document.
//XmlElement root = doc.DocumentElement;
//doc.InsertBefore(xmldecl, root);





//XmlDeclaration xmldecl;
//xmldecl = doc.CreateXmlDeclaration("1.0", null, null);
//xmldecl.Encoding = "UTF-8";


/*XmlDocument doc = new XmlDocument();
string xmlString = "<book><title>Oberon's Legacy</title></book>";
doc.Load(new StringReader(xmlString));

string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML12.xml";
// Create an XML declaration. 
XmlDeclaration xmldecl;
xmldecl = doc.CreateXmlDeclaration("1.0", null, null);
xmldecl.Encoding = "UTF-8";
//xmldecl.Standalone = "yes";

// Add the new node to the document.
XmlElement root = doc.DocumentElement;
doc.InsertBefore(xmldecl, root);

// Display the modified XML document 
//Console.WriteLine(doc.OuterXml);*/



//XmlDocument xDoc = new XmlDocument();

//String headerForXml = "<? xml version = '1.0' encoding = 'utf-8' ?> ";
//byte[] bytes = Encoding.Default.GetBytes(headerForXml);
//headerForXml = Encoding.UTF8.GetString(bytes);





/*string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML12.xml";
XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);



XmlWriterSettings settings = new XmlWriterSettings();
settings.OmitXmlDeclaration = true;




writer.Formatting = Formatting.Indented;
xDoc.Save(writer);*/


/*XmlDeclaration xDecl = xDoc.CreateXmlDeclaration("1.0", null, null);
if (xDoc.FirstChild.NodeType != null)
{
    if (xDoc.FirstChild.NodeType == XmlNodeType.XmlDeclaration)
    {
        xDoc.ReplaceChild(xDecl, xDoc.FirstChild);
    }
    else
    {
        xDoc.InsertBefore(xDecl, xDoc.DocumentElement);
    }
}*/



/*XmlDocument doc = new XmlDocument();
// Create a procesing instruction.
XmlProcessingInstruction newPI;
String PItext = "type='text/xsl' href='book.xsl'";
newPI = doc.CreateProcessingInstruction("xml-stylesheet", PItext);
// Display the target and data information.
//Console.WriteLine("<?{0} {1}?>", newPI.Target, newPI.Data);
// Add the processing instruction node to the document.
doc.AppendChild(newPI);*/


/*XmlDocument doc = new XmlDocument();
XmlProcessingInstruction newPI;
String PItext = "type='text/xsl' href='book.xsl'";
newPI = doc.CreateProcessingInstruction("xml-stylesheet", PItext);

string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML10.xml";
XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
//XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);

createNode("1", "Product 1", "1000", writer);
createNode("2", "Product 2", "2000", writer);
createNode("3", "Product 3", "3000", writer);
createNode("4", "Product 4", "4000", writer);*/


/*writer.WriteStartDocument(true);
writer.Formatting = Formatting.Indented;
writer.Indentation = 2;
writer.WriteStartElement("Table");
createNode("1", "Product 1", "1000", writer);
createNode("2", "Product 2", "2000", writer);
createNode("3", "Product 3", "3000", writer);
createNode("4", "Product 4", "4000", writer);
writer.WriteEndElement();
writer.WriteEndDocument();
writer.Close();
MessageBox.Show("XML File created ! ");*/


/*// Create the XmlDocument.
XmlDocument doc = new XmlDocument();
doc.LoadXml("<item><name>wrench</name></item>"); //Your string here

string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\XMLTEST01.xml";

// Save the document to a file and auto-indent the output.
XmlTextWriter writer = new XmlTextWriter(path, null);
writer.Formatting = Formatting.Indented;
doc.Save(writer);*/







/*private void button1_Click(object sender, EventArgs e)
{
    string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML.xml";
    XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
    writer.WriteStartDocument(true);
    writer.Formatting = Formatting.Indented;
    writer.Indentation = 2;
    writer.WriteStartElement("Table");
    createNode("1", "Product 1", "1000", writer);
    createNode("2", "Product 2", "2000", writer);
    createNode("3", "Product 3", "3000", writer);
    createNode("4", "Product 4", "4000", writer);
    writer.WriteEndElement();
    writer.WriteEndDocument();
    writer.Close();
    MessageBox.Show("XML File created ! ");


    // Create the XmlDocument.
    XmlDocument doc = new XmlDocument();
    doc.LoadXml("<item><name>wrench</name></item>"); //Your string here

    string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\XMLTEST01.xml";

    // Save the document to a file and auto-indent the output.
    XmlTextWriter writer = new XmlTextWriter(path, null);
    writer.Formatting = Formatting.Indented;
    doc.Save(writer);

}
private void createNode(string pID, string pName, string pPrice, XmlTextWriter writer)
{
    writer.WriteStartElement("Product");
    writer.WriteStartElement("Product_id");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("Product_name");
    writer.WriteString(pName);
    writer.WriteEndElement();
    writer.WriteStartElement("Product_price");
    writer.WriteString(pPrice);
    writer.WriteEndElement();
    writer.WriteEndElement();
}*/













//doc.Save("c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\");
//doc.Save("c:\\");


//XmlTextWriter writer = new XmlTextWriter(path, null);





/*XmlDocument doc = new XmlDocument();
XmlNode docNode = doc.CreateXmlDeclaration("1.0", "UTF-8", null);
doc.AppendChild(docNode);

XmlNode productsNode = doc.CreateElement("products");
doc.AppendChild(productsNode);

XmlNode productNode = doc.CreateElement("product");
XmlAttribute productAttribute = doc.CreateAttribute("id");
productAttribute.Value = "01";
productNode.Attributes.Append(productAttribute);
productsNode.AppendChild(productNode);

XmlNode nameNode = doc.CreateElement("Name");
nameNode.AppendChild(doc.CreateTextNode("Java"));
productNode.AppendChild(nameNode);
XmlNode priceNode = doc.CreateElement("Price");
priceNode.AppendChild(doc.CreateTextNode("Free"));
productNode.AppendChild(priceNode);

// Create and add another product node.
productNode = doc.CreateElement("product");
productAttribute = doc.CreateAttribute("id");
productAttribute.Value = "02";
productNode.Attributes.Append(productAttribute);
productsNode.AppendChild(productNode);
nameNode = doc.CreateElement("Name");
nameNode.AppendChild(doc.CreateTextNode("C#"));
productNode.AppendChild(nameNode);
priceNode = doc.CreateElement("Price");
priceNode.AppendChild(doc.CreateTextNode("Free"));
productNode.AppendChild(priceNode);

doc.Save(Console.Out);     */









/*private void button1_Click(object sender, EventArgs e)
{


    string path = "c:\\Users\\ninekorn\\Desktop\\XMLTESTS\\testingXML20.xml";

    XmlWriterSettings settings = new XmlWriterSettings();
    settings.Encoding = Encoding.UTF8;
    settings.Indent = true;
    XmlTextWriter writer = new XmlTextWriter(path, System.Text.Encoding.UTF8);
    //writer.WriteStartDocument(true);
    writer.WriteProcessingInstruction("xml", "version=\"1.0\" encoding=\"UTF-8\"");

    writer.Formatting = Formatting.Indented;
    writer.Indentation = 2;

    writer.WriteStartElement("root");
    createNodeHEADER("1000", "1000", "1000", writer);
    writer.WriteEndElement();
    //writer.WriteEndDocument();
    writer.Close();
    MessageBox.Show("XML File created !");
}

private void createNodeHEADER(string pID, string pName, string pPrice, XmlTextWriter writer)
{
    writer.WriteStartElement("header");
    writer.WriteString(pID);

    writer.WriteStartElement("id");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("title");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("description");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("enabled");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteEndElement();

    /*writer.WriteStartElement("gfx");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("description");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("enabled");
    writer.WriteString(pID);
    //writer.WriteEndElement();
    writer.WriteEndElement();
}


private void createNodeGFX(string pID, string pName, string pPrice, XmlTextWriter writer)
{

    writer.WriteStartElement("id");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("title");
    writer.WriteString(pID);
    writer.WriteEndElement();
    writer.WriteStartElement("description");
    writer.WriteString(pName);
    writer.WriteEndElement();
    writer.WriteStartElement("enabled");
    writer.WriteString(pPrice);
    //writer.WriteEndElement();
    writer.WriteEndElement();
}
*/
