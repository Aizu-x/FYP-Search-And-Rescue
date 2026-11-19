// MakeIcon: turns a normal picture (launcher\icon.png) into the Windows icon that is built into FYP_SRM.exe.
//
// Windows wants the icon of a program in its own format (.ico, the same picture in several sizes) and stores it
// inside the .exe. So a new design is used like this: replace launcher\icon.png, run launcher\BUILD.cmd.
// BUILD.cmd compiles this file, runs it and deletes the result again; nothing has to be installed.
//
//   MakeIcon <picture.png> <out.ico>     make the icon (16 to 256 pixels) from the picture
//   MakeIcon --refresh <file>            tell Windows Explorer that this file's icon may have changed

using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Drawing.Imaging;
using System.IO;
using System.Runtime.InteropServices;

static class MakeIcon
{
    static readonly int[] Sizes = { 16, 20, 24, 32, 40, 48, 64, 128, 256 };

    static int Main(string[] args)
    {
        try
        {
            if (args.Length == 2 && args[0] == "--refresh")
            {
                // SHCNE_UPDATEITEM for this path: Explorer reads the file's icon again instead of showing a remembered one
                SHChangeNotify(0x00002000, 0x0005 | 0x1000, Path.GetFullPath(args[1]), IntPtr.Zero);
                return 0;
            }
            if (args.Length != 2)
            {
                Console.Error.WriteLine("usage: MakeIcon <picture.png> <out.ico>");
                return 2;
            }
            using (Bitmap source = Load(args[0]))
            {
                if (source.Width < 256 || source.Height < 256)
                    Console.WriteLine("Note: the picture is only " + source.Width + " x " + source.Height
                                      + " pixels. 256 x 256 or larger looks sharper.");
                if (source.Width != source.Height)
                    Console.WriteLine("Note: the picture is not square. It is centred on a square with see-through edges.");
                List<byte[]> images = new List<byte[]>();
                foreach (int size in Sizes)
                {
                    using (Bitmap scaled = Scale(source, size))
                        images.Add(size >= 256 ? Png(scaled) : Dib(scaled));
                }
                Write(args[1], images);
                Console.WriteLine("Icon made from " + Path.GetFileName(args[0]) + " (" + source.Width + " x "
                                  + source.Height + " pixels), in " + Sizes.Length + " sizes.");
            }
            return 0;
        }
        catch (Exception error)
        {
            Console.Error.WriteLine("Could not make the icon from " + (args.Length > 0 ? args[0] : "?") + ": " + error.Message);
            Console.Error.WriteLine("The picture must be a PNG (or JPG / BMP) file that an image viewer can open.");
            return 1;
        }
    }

    static Bitmap Load(string path)
    {
        using (MemoryStream stream = new MemoryStream(File.ReadAllBytes(path)))
        using (Image image = Image.FromStream(stream))
        {
            Bitmap copy = new Bitmap(image.Width, image.Height, PixelFormat.Format32bppArgb);
            using (Graphics g = Graphics.FromImage(copy))
            {
                g.CompositingMode = CompositingMode.SourceCopy;
                g.DrawImage(image, new Rectangle(0, 0, image.Width, image.Height));
            }
            return copy;
        }
    }

    /// The picture at size x size pixels: whole picture visible, centred, see-through where it does not reach.
    static Bitmap Scale(Bitmap source, int size)
    {
        Bitmap result = new Bitmap(size, size, PixelFormat.Format32bppArgb);
        double factor = Math.Min((double)size / source.Width, (double)size / source.Height);
        int width = Math.Max(1, (int)Math.Round(source.Width * factor));
        int height = Math.Max(1, (int)Math.Round(source.Height * factor));
        using (Graphics g = Graphics.FromImage(result))
        using (ImageAttributes edges = new ImageAttributes())
        {
            g.Clear(Color.Transparent);
            g.CompositingQuality = CompositingQuality.HighQuality;
            g.InterpolationMode = InterpolationMode.HighQualityBicubic;
            g.PixelOffsetMode = PixelOffsetMode.HighQuality;
            edges.SetWrapMode(WrapMode.TileFlipXY);         // no darker line along the picture's edges
            g.DrawImage(source, new Rectangle((size - width) / 2, (size - height) / 2, width, height),
                        0, 0, source.Width, source.Height, GraphicsUnit.Pixel, edges);
        }
        return result;
    }

    static byte[] Png(Bitmap image)
    {
        using (MemoryStream stream = new MemoryStream())
        {
            image.Save(stream, ImageFormat.Png);
            return stream.ToArray();
        }
    }

    /// One icon picture the classic way: a bitmap header, the pixels (blue, green, red, alpha; bottom row first),
    /// then a one-bit mask that marks the fully see-through pixels.
    static byte[] Dib(Bitmap image)
    {
        int size = image.Width;
        int maskRow = ((size + 31) / 32) * 4;
        byte[] pixels = new byte[size * size * 4];
        BitmapData data = image.LockBits(new Rectangle(0, 0, size, size), ImageLockMode.ReadOnly,
                                         PixelFormat.Format32bppArgb);
        try
        {
            for (int y = 0; y < size; y++)
                Marshal.Copy(new IntPtr(data.Scan0.ToInt64() + (long)y * data.Stride), pixels, y * size * 4, size * 4);
        }
        finally
        {
            image.UnlockBits(data);
        }
        using (MemoryStream stream = new MemoryStream())
        using (BinaryWriter writer = new BinaryWriter(stream))
        {
            writer.Write(40);                               // BITMAPINFOHEADER
            writer.Write(size);
            writer.Write(size * 2);                         // pixels + mask
            writer.Write((short)1);
            writer.Write((short)32);
            writer.Write(0);
            writer.Write(size * size * 4 + maskRow * size);
            writer.Write(0);
            writer.Write(0);
            writer.Write(0);
            writer.Write(0);
            for (int y = size - 1; y >= 0; y--)
                writer.Write(pixels, y * size * 4, size * 4);
            for (int y = size - 1; y >= 0; y--)
            {
                byte[] row = new byte[maskRow];
                for (int x = 0; x < size; x++)
                {
                    if (pixels[(y * size + x) * 4 + 3] == 0)
                        row[x / 8] |= (byte)(0x80 >> (x % 8));
                }
                writer.Write(row);
            }
            writer.Flush();
            return stream.ToArray();
        }
    }

    static void Write(string path, List<byte[]> images)
    {
        using (FileStream file = new FileStream(path, FileMode.Create, FileAccess.Write))
        using (BinaryWriter writer = new BinaryWriter(file))
        {
            writer.Write((short)0);
            writer.Write((short)1);                         // 1 = icon
            writer.Write((short)images.Count);
            int offset = 6 + 16 * images.Count;
            for (int i = 0; i < images.Count; i++)
            {
                int size = Sizes[i];
                writer.Write((byte)(size >= 256 ? 0 : size));       // 0 means 256
                writer.Write((byte)(size >= 256 ? 0 : size));
                writer.Write((byte)0);
                writer.Write((byte)0);
                writer.Write((short)1);
                writer.Write((short)32);
                writer.Write(images[i].Length);
                writer.Write(offset);
                offset += images[i].Length;
            }
            foreach (byte[] image in images)
                writer.Write(image);
        }
    }

    [DllImport("shell32.dll", CharSet = CharSet.Unicode)]
    static extern void SHChangeNotify(int eventId, uint flags, string item, IntPtr unused);
}
