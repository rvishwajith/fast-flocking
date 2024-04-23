/// FrameRateCounter.cs
/// Author: Rohith Vishwajith
/// Created 4/22/2024

using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

/// <summary>
/// FPSCounter:
/// A component used to calculate and display the frame rate of the player on a Label.
/// Based off: https://forum.unity.com/threads/fps-counter.505495/
/// </summary>
public class FPSCounter : MonoBehaviour
{
    private Dictionary<int, string> cachedNumberStrings = new();
    private int[] frameRateSamples;
    private int cacheNumbersAmount = 300;
    private int averageFromAmount = 30;
    private int averageCounter = 0;
    private int currentAveraged;

    void Awake()
    {
        for (int i = 0; i < cacheNumbersAmount; i++)
            cachedNumberStrings[i] = i.ToString();
        frameRateSamples = new int[averageFromAmount];
    }

    void Update()
    {
        // Sample
        var currentFrame = (int)Math.Round(1f / Time.smoothDeltaTime); // If your game modifies Time.timeScale, use unscaledDeltaTime and smooth manually (or not).
        frameRateSamples[averageCounter] = currentFrame;

        // Average
        var average = 0f;
        foreach (var frameRate in frameRateSamples)
            average += frameRate;
        currentAveraged = Mathf.RoundToInt(average / averageFromAmount);
        averageCounter = (averageCounter + 1) % averageFromAmount;

        // Assign to UI
        var text = currentAveraged switch
        {
            var x when x >= 0 && x < cacheNumbersAmount => cachedNumberStrings[x],
            var x when x >= cacheNumbersAmount => $"> {cacheNumbersAmount}",
            var x when x < 0 => "< 0",
            _ => "?"
        };
    }
}