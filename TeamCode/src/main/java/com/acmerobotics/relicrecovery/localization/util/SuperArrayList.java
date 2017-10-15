package com.acmerobotics.relicrecovery.localization.util;

import java.util.ArrayList;

/**
 * I have the best array lists.
 * I have to say a lot of people have been asking this question.
 * No, really. A lot of people come up to me and they ask me.
 * They say, "What's the best Array List? And I tell them look, we know what the best Array List is.
 * We've had almost eight years of the worst kind of data structures you can imagine.
 * Oh my god, I can't believe it. Adding to and removing from the arrays the stacks and the heaps.
 * Its terrible. Its just terrible. Look, if you want to know what the best array list is, do you want to know what the best array list is?
 * I'll tell you. First of all the language java, by the way I love the language java.
 * It's probably my favorite language, no it is my favorite language.
 * You know what, it's probably more like the language java but with a lot of other JVM languages.
 * A lot. If I'm being honest, I mean, if I'm being honest. I like a lot of JVM languages.
 * Except for Marco Rubio, now he's a JVM language that I don't like. Though, I probably shouldn't say that.
 * He's a nice guy but he's like, "((())())(())))((())))))))))", on and on, like that. He's like lisp! You know what I mean?
 * He's like lisp. I don't know. I mean, you know. So, we have all these data structures and we can add to them and subtract from them and add to them.
 * COMBINE them even. Did you know that?
 * We can combine them OR split them, they don't tell you that, and I'll tell you, no one is better at array operations than me.
 * You wouldn't believe it. That I can tell you. So, we're gonna be the best on Array Lists, believe me. OK? Alright.
 * You know it, I know it, we all know it, I have the best Array Lists.
 * These people, they keep calling me on the phone, they say "Kelly, you have the best array lists."
 * And I tell them, yes I know, but you know, I am a very humble guy, maybe the most humble guy, with really huge hands though.
 * And these array Lists I have, well, they are the best array lists. They are going to be HUGE.
 * We are going to make array lists GREAT again.
 * If you listen to those guys over at FAKE NEWS Oracle, they say, you know, that we already have good enough array lists, but they are liars, and, frankly, losers.
 * ok, ok, thank you, ok, goodnight, ok.
 */

public class SuperArrayList<E> extends ArrayList<E> {

    @Override
    public E get(int index) {
        if (index < 0) {
            return super.get(this.size() + index);
        }
        return super.get(index);
    }

}
