//using System;
using System.Collections.Generic;
//using System.Text;

//using SharpDX.DirectSound;
using SharpDX.XAudio2;
using SharpDX.Multimedia;
//using SharpDX;
using SharpDX.IO;

//using System.Globalization;
//using System.IO;
//using System.Runtime.InteropServices;


namespace sccsVD4VE_LightNWithoutVr
{
    public class SoundPlayer
    {
        class MyWave
        {
            public AudioBuffer Buffer { get; set; }
            public uint[] DecodedPacketsInfo { get; set; }
            public WaveFormat WaveFormat { get; set; }
        }

        private XAudio2 xaudio;
        private MasteringVoice mvoice;
        Dictionary<string, MyWave> sounds;

        // Constructor
        public SoundPlayer()
        {
            xaudio = new XAudio2();
            xaudio.StartEngine();
            mvoice = new MasteringVoice(xaudio);
            sounds = new Dictionary<string, MyWave>();
        }

        // Reads a sound and puts it in the dictionary
        public AudioBuffer AddWave(string key, string filepath)
        {
            MyWave wave = new MyWave();

            var nativeFileStream = new NativeFileStream(filepath, NativeFileMode.Open, NativeFileAccess.Read, NativeFileShare.Read);
            var soundStream = new SoundStream(nativeFileStream);
            var buffer = new AudioBuffer() { Stream = soundStream, AudioBytes = (int)soundStream.Length, Flags = SharpDX.XAudio2.BufferFlags.EndOfStream };

            wave.Buffer = buffer;
            wave.DecodedPacketsInfo = soundStream.DecodedPacketsInfo;
            wave.WaveFormat = soundStream.Format;

            this.sounds.Add(key, wave);
            return buffer;
        }




        // Plays the sound
        public void Play(string key)
        {
            if (!this.sounds.ContainsKey(key))
            {
                return;
            }

            MyWave w = this.sounds[key];

            var sourceVoice = new SourceVoice(this.xaudio, w.WaveFormat);
            sourceVoice.SubmitSourceBuffer(w.Buffer, w.DecodedPacketsInfo);
            sourceVoice.Start();
        }
    }
}
